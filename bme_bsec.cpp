// BME688 + Bosch BSEC2. See bme_bsec.h.
//
// The BSEC call sequence follows Bosch's Arduino wrapper (Bosch-BSEC2-Library
// src/bsec2.cpp, v1.10.2610), with one deliberate difference: in forced mode
// the wrapper reads the sensor right after starting a measurement, so it
// actually processes the previous cycle's result. Here the caller waits the
// measurement duration (start() returns it) before finish() reads the data,
// as in Bosch's older reference integration (bsec_integration.c).
#include "bme_bsec.h"

#include <string.h>
#include "mbed.h"
#include "flc.h"
#include "icc.h"
#include "bme68x.h"
#include "bsec_interface.h"
#include "bsec_datatypes.h"
#include "bess_proto.h"   // crc16_ccitt

namespace bmebsec {

// ---- BSEC configuration ---------------------------------------------------------
// 1: bme688_sel_33v_3s_4d (BME688, 3.3 V, 3 s, 4-day history). Matches LP: subscribe -> 0.
// 0: the library's built-in default config, as in Bosch's basic example. That one
//    is meant for ULP (300 s): subscribing at LP returns warning 14 (sample-rate mismatch).
// Measured with both: after every boot BSEC runs a 7-cycle heater run-in
// (380 C x4, 150 C, 200 C x2, 1 s each), then holds 320 C for 197 ms. The frames
// carry the heater temperature so analysis can keep the 320 C readings only.
#ifndef BESS_BSEC_SEL_CONFIG
#define BESS_BSEC_SEL_CONFIG 1
#endif
static const uint8_t BSEC_CONFIG[] = {
#include "bsec2/config/bme688_sel_33v_3s_4d/bsec_selectivity.txt"
};

// Bosch-BSEC2-Library bsec2.h: total heater time per parallel-mode step [ms].
static const uint16_t BSEC_TOTAL_HEAT_DUR = 140;

#ifndef BESS_BSEC_SAVE_PERIOD_S
#define BESS_BSEC_SAVE_PERIOD_S (6 * 3600)  // as Bosch's example: 4 saves a day
#endif
static const float STATE_SAVE_PERIOD_S = BESS_BSEC_SAVE_PERIOD_S;

// ---- I2C glue for the Bosch driver -------------------------------------------
static I2C *s_bus;
static int s_addr8;

static BME68X_INTF_RET_TYPE i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *)
{
    if (s_bus->write(s_addr8, (const char *)&reg, 1, true) != 0) return -1;
    return s_bus->read(s_addr8, (char *)data, (int)len) == 0 ? 0 : -1;
}

// Bosch's driver passes the first register here and the rest interleaved in
// data (reg, value, reg, value ...), so it all goes out as one burst.
static BME68X_INTF_RET_TYPE i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *)
{
    char buf[1 + BME68X_LEN_INTERLEAVE_BUFF];
    if (len > sizeof buf - 1) return -1;
    buf[0] = (char)reg;
    memcpy(buf + 1, data, len);
    return s_bus->write(s_addr8, buf, (int)len + 1) == 0 ? 0 : -1;
}

static void delay_us(uint32_t us, void *) { wait_us((int)us); }

// ---- state -------------------------------------------------------------------
static struct bme68x_dev s_dev;
static bool s_bsec_ok;
static bool s_custom_config;
static uint8_t s_work[BSEC_MAX_WORKBUFFER_SIZE];
static bsec_bme_settings_t s_set;
static int64_t s_next_call;
static int64_t s_meas_ts;           // timestamp handed to BSEC for this cycle's data
static uint8_t s_meas_mode = BME68X_SLEEP_MODE;   // what this cycle measured
static uint8_t s_sensor_mode = BME68X_SLEEP_MODE; // what the sensor is configured for
static uint16_t s_prof_temp[10], s_prof_dur[10];  // parallel-mode profile (driver keeps pointers)
static int s_cycle_status;

bool bsec_running() { return s_bsec_ok; }
uint8_t variant() { return (uint8_t)s_dev.variant_id; }
int64_t next_call_ns() { return s_next_call; }

// ---- BSEC state in the last flash page ----------------------------------------
// pyocd erases only the pages it programs, so this page survives reflashing.
static const uint32_t STATE_ADDR = MXC_FLASH_FULL_MEM_SIZE - MXC_FLASH_PAGE_SIZE;  // 0x1FE000
static const uint32_t STATE_MAGIC = 0x32455342;  // "BSE2"

struct alignas(4) StateRecord {
    uint32_t magic;
    uint16_t format;      // 1
    uint16_t len;         // bytes of state used
    uint16_t crc;         // CRC-16/CCITT-FALSE over state[0..len)
    uint16_t reserved;
    uint32_t saves;       // how many times this page has been written
    uint8_t state[240];   // >= BSEC_MAX_STATE_BLOB_SIZE (238)
};
static_assert(sizeof(StateRecord) % 4 == 0, "flash writes are whole words");
static_assert(sizeof(((StateRecord *)0)->state) >= BSEC_MAX_STATE_BLOB_SIZE, "state too small");

extern "C" uint32_t __etext, __data_start__, __data_end__;

static const StateRecord *flash_record()
{
    const StateRecord *r = (const StateRecord *)STATE_ADDR;
    if (r->magic != STATE_MAGIC || r->format != 1 || r->len == 0 || r->len > BSEC_MAX_STATE_BLOB_SIZE) return nullptr;
    if (bess::crc16_ccitt((const char *)r->state, r->len) != r->crc) return nullptr;
    return r;
}

static void load_state()
{
    const StateRecord *r = flash_record();
    if (!r) {
        printf("BSEC state: none saved in flash (calibration starts from scratch)\r\n");
        return;
    }
    bsec_library_return_t b = bsec_set_state(r->state, r->len, s_work, sizeof s_work);
    printf("BSEC state: loaded %u bytes from flash (save #%lu) -> %d\r\n", r->len, (unsigned long)r->saves, (int)b);
}

static bool save_state()
{
    static StateRecord rec;
    uint32_t n = 0;
    memset(&rec, 0xFF, sizeof rec);
    bsec_library_return_t b = bsec_get_state(0, rec.state, BSEC_MAX_STATE_BLOB_SIZE, s_work, sizeof s_work, &n);
    if (b != BSEC_OK || n == 0 || n > BSEC_MAX_STATE_BLOB_SIZE) {
        printf("BSEC state: get_state failed (%d, %lu bytes)\r\n", (int)b, (unsigned long)n);
        return false;
    }
    const StateRecord *old = flash_record();
    rec.magic = STATE_MAGIC;
    rec.format = 1;
    rec.len = (uint16_t)n;
    rec.crc = bess::crc16_ccitt((const char *)rec.state, n);
    rec.reserved = 0;
    rec.saves = old ? old->saves + 1 : 1;
    if (old && old->len == rec.len && memcmp(old->state, rec.state, n) == 0) return false;  // unchanged

    // Never touch flash the program occupies.
    uint32_t image_end = (uint32_t)&__etext + ((uint32_t)&__data_end__ - (uint32_t)&__data_start__);
    if (image_end > STATE_ADDR) {
        printf("BSEC state: image reaches 0x%08lX, not saving over it\r\n", (unsigned long)image_end);
        return false;
    }
    // Reads (including instruction fetches) stall while the controller works,
    // so this is safe to run from flash. A page erase takes tens of ms.
    int e = FLC_Init();
    if (e == E_NO_ERROR) e = FLC_PageErase(STATE_ADDR, MXC_V_FLC_ERASE_CODE_PAGE_ERASE, MXC_V_FLC_FLSH_UNLOCK_KEY);
    if (e == E_NO_ERROR) e = FLC_Write(STATE_ADDR, &rec, sizeof rec, MXC_V_FLC_FLSH_UNLOCK_KEY);
    ICC_Flush();  // drop cached copies of the old page
    bool ok = e == E_NO_ERROR && memcmp((const void *)STATE_ADDR, &rec, sizeof rec) == 0;
    printf("BSEC state: %s %lu bytes to flash at 0x%08lX (save #%lu, err %d)\r\n", ok ? "saved" : "FAILED saving",
           (unsigned long)n, (unsigned long)STATE_ADDR, (unsigned long)rec.saves, e);
    return ok;
}

// ---- bring-up -----------------------------------------------------------------
static bsec_library_return_t subscribe()
{
    static const uint8_t ids[] = {
        BSEC_OUTPUT_IAQ, BSEC_OUTPUT_STATIC_IAQ, BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT, BSEC_OUTPUT_STABILIZATION_STATUS, BSEC_OUTPUT_RUN_IN_STATUS,
    };
    bsec_sensor_configuration_t req[sizeof ids], phys[BSEC_MAX_PHYSICAL_SENSOR];
    for (size_t i = 0; i < sizeof ids; i++) {
        req[i].sensor_id = ids[i];
        req[i].sample_rate = BSEC_SAMPLE_RATE_LP;
    }
    uint8_t n_phys = BSEC_MAX_PHYSICAL_SENSOR;
    return bsec_update_subscription(req, sizeof ids, phys, &n_phys);
}

static bool bsec_bringup(bool custom_config)
{
    bsec_library_return_t b = bsec_init();
    if (b != BSEC_OK) {
        printf("BSEC: init -> %d\r\n", (int)b);
        return false;
    }
    if (custom_config) {
        b = bsec_set_configuration(BSEC_CONFIG, sizeof BSEC_CONFIG, s_work, sizeof s_work);
        printf("BSEC: config bme688_sel_33v_3s_4d (%u bytes) -> %d\r\n", (unsigned)sizeof BSEC_CONFIG, (int)b);
        if (b < 0) return false;
    } else {
        printf("BSEC: using the library's built-in default config\r\n");
    }
    load_state();
    b = subscribe();
    printf("BSEC: subscribe IAQ, static IAQ, CO2-eq, bVOC-eq, status @ LP (3 s) -> %d\r\n", (int)b);
    return b >= 0;
}

bool begin(I2C &bus, uint8_t addr7, int64_t now_ns)
{
    s_bus = &bus;
    s_addr8 = addr7 << 1;
    memset(&s_dev, 0, sizeof s_dev);
    s_dev.intf = BME68X_I2C_INTF;
    s_dev.read = i2c_read;
    s_dev.write = i2c_write;
    s_dev.delay_us = delay_us;
    s_dev.amb_temp = 25;
    int8_t r = bme68x_init(&s_dev);
    printf("BME68x (Bosch API v4.4.8) at 0x%02X: init -> %d, chip_id=0x%02X variant=0x%02lX (%s)\r\n", addr7, r,
           s_dev.chip_id, (unsigned long)s_dev.variant_id,
           s_dev.variant_id == BME68X_VARIANT_GAS_HIGH ? "BME688" : "BME680");
    if (r != BME68X_OK) return false;
    s_sensor_mode = BME68X_SLEEP_MODE;

    bsec_version_t v;
    bsec_get_version(&v);
    printf("BSEC v%u.%u.%u.%u\r\n", v.major, v.minor, v.major_bugfix, v.minor_bugfix);
    s_custom_config = BESS_BSEC_SEL_CONFIG;
    s_bsec_ok = bsec_bringup(s_custom_config);
    if (!s_bsec_ok && s_custom_config) {
        printf("BSEC: BME688 config rejected, retrying with the default config\r\n");
        s_custom_config = false;
        s_bsec_ok = bsec_bringup(false);
    }
    if (!s_bsec_ok) printf("BSEC: NOT running. Plain 320 C / 150 ms measurements every 3 s instead.\r\n");
    s_next_call = now_ns;
    return true;
}

// ---- one cycle ----------------------------------------------------------------
int32_t start(int64_t now_ns, float amb_c)
{
    s_cycle_status = 0;
    if (amb_c > 60) amb_c = 60;
    if (amb_c < -40) amb_c = -40;
    s_dev.amb_temp = (int8_t)(amb_c + (amb_c < 0 ? -0.5f : 0.5f));  // Bosch takes whole degrees

    if (s_bsec_ok) {
        bsec_library_return_t b = bsec_sensor_control(now_ns, &s_set);
        s_cycle_status = b;
        if (b < 0) {
            // Shouldn't happen once subscribed; try again one interval later.
            s_next_call = now_ns + CYCLE_NS;
            s_meas_mode = BME68X_SLEEP_MODE;
            return 0;
        }
        s_next_call = s_set.next_call;
    } else {
        memset(&s_set, 0, sizeof s_set);
        s_set.heater_temperature = 320;
        s_set.heater_duration = 150;
        s_set.run_gas = 1;
        s_set.temperature_oversampling = BME68X_OS_2X;
        s_set.pressure_oversampling = BME68X_OS_16X;
        s_set.humidity_oversampling = BME68X_OS_2X;
        s_set.trigger_measurement = 1;
        s_set.op_mode = BME68X_FORCED_MODE;
        s_next_call += CYCLE_NS;
        if (s_next_call <= now_ns) s_next_call = now_ns + CYCLE_NS;  // fell behind: don't burst
    }
    s_meas_ts = now_ns;

    // BSEC sets oversampling 0 ("skip") for channels it doesn't need; the
    // sensor then returns a placeholder ADC value that converts to nonsense
    // (pressure read 659 hPa). Keep at least 1x so every frame field is real.
    // That adds only a few ms to the measurement.
    auto at_least_1x = [](uint8_t os) { return os == BME68X_OS_NONE ? (uint8_t)BME68X_OS_1X : os; };
    struct bme68x_conf conf;
    conf.os_hum = at_least_1x(s_set.humidity_oversampling);
    conf.os_temp = at_least_1x(s_set.temperature_oversampling);
    conf.os_pres = at_least_1x(s_set.pressure_oversampling);
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    int8_t r = BME68X_OK;

    if (!s_set.trigger_measurement || s_set.op_mode == BME68X_SLEEP_MODE) {
        if (s_sensor_mode != BME68X_SLEEP_MODE) r = bme68x_set_op_mode(BME68X_SLEEP_MODE, &s_dev);
        s_sensor_mode = s_meas_mode = BME68X_SLEEP_MODE;
        return r < 0 ? r : 0;
    }

    if (s_set.op_mode == BME68X_FORCED_MODE) {
        struct bme68x_heatr_conf h = {};
        h.enable = s_set.run_gas ? BME68X_ENABLE : BME68X_DISABLE;
        h.heatr_temp = s_set.heater_temperature;
        h.heatr_dur = s_set.heater_duration;
        r = bme68x_set_conf(&conf, &s_dev);
        if (r == BME68X_OK) r = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &h, &s_dev);
        if (r == BME68X_OK) r = bme68x_set_op_mode(BME68X_FORCED_MODE, &s_dev);
        if (r != BME68X_OK) return r;
        s_sensor_mode = s_meas_mode = BME68X_FORCED_MODE;
        uint32_t us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &s_dev);
        if (s_set.run_gas) us += (uint32_t)s_set.heater_duration * 1000u;
        return (int32_t)us;
    }

    // Parallel mode: the sensor runs its heater profile continuously; each
    // cycle collects whatever fields are ready (Bosch wrapper behaviour).
    if (s_sensor_mode != BME68X_PARALLEL_MODE) {
        r = bme68x_set_conf(&conf, &s_dev);
        if (r != BME68X_OK) return r;
        uint8_t len = s_set.heater_profile_len > 10 ? 10 : s_set.heater_profile_len;
        memcpy(s_prof_temp, s_set.heater_temperature_profile, sizeof s_prof_temp);
        memcpy(s_prof_dur, s_set.heater_duration_profile, sizeof s_prof_dur);
        struct bme68x_heatr_conf h = {};
        h.enable = BME68X_ENABLE;
        h.heatr_temp_prof = s_prof_temp;
        h.heatr_dur_prof = s_prof_dur;
        h.profile_len = len;
        h.shared_heatr_dur =
            (uint16_t)(BSEC_TOTAL_HEAT_DUR - bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &s_dev) / 1000u);
        r = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &h, &s_dev);
        if (r == BME68X_OK) r = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &s_dev);
        if (r != BME68X_OK) return r;
        s_sensor_mode = BME68X_PARALLEL_MODE;
        printf("BME68x: parallel mode, %u-step heater profile\r\n", len);
    }
    s_meas_mode = BME68X_PARALLEL_MODE;
    return 0;
}

static void run_bsec(const struct bme68x_data &d, Reading &out)
{
    bsec_input_t in[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n = 0;
    uint32_t want = s_set.process_data;
    auto add = [&](uint8_t id, float v) {
        if ((want & (1u << (id - 1))) && n < BSEC_MAX_PHYSICAL_SENSOR) {
            in[n].sensor_id = id;
            in[n].signal = v;
            in[n].signal_dimensions = 1;
            in[n].time_stamp = s_meas_ts;
            n++;
        }
    };
    add(BSEC_INPUT_HEATSOURCE, 0.0f);   // no extra device heating to compensate; TMP117 gives air temperature
    add(BSEC_INPUT_TEMPERATURE, d.temperature);
    add(BSEC_INPUT_HUMIDITY, d.humidity);
    // hPa: Bosch's wrapper converts Pa -> hPa before BSEC since BSEC 2.5
    // (commit 71b723f), although bsec_datatypes.h still says Pa.
    add(BSEC_INPUT_PRESSURE, d.pressure * 0.01f);
    add(BSEC_INPUT_GASRESISTOR, d.gas_resistance);
    add(BSEC_INPUT_PROFILE_PART, s_meas_mode == BME68X_FORCED_MODE ? 0.0f : (float)d.gas_index);
    if (!n) return;

    bsec_output_t o[BSEC_NUMBER_OUTPUTS];
    uint8_t n_out = BSEC_NUMBER_OUTPUTS;
    bsec_library_return_t b = bsec_do_steps(in, n, o, &n_out);
    if (b != BSEC_OK) s_cycle_status = b;
    if (b < 0) return;
    for (uint8_t i = 0; i < n_out; i++) {
        switch (o[i].sensor_id) {
        case BSEC_OUTPUT_IAQ:
            out.iaq = o[i].signal;
            out.iaq_acc = o[i].accuracy;
            out.bsec_out = true;
            break;
        case BSEC_OUTPUT_STATIC_IAQ: out.siaq = o[i].signal; break;
        case BSEC_OUTPUT_CO2_EQUIVALENT: out.co2eq = o[i].signal; break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT: out.bvoc = o[i].signal; break;
        case BSEC_OUTPUT_STABILIZATION_STATUS: out.stab = o[i].signal > 0.5f; break;
        case BSEC_OUTPUT_RUN_IN_STATUS: out.run_in = o[i].signal > 0.5f; break;
        default: break;
        }
    }
}

int finish(Reading &out)
{
    out = Reading();
    out.heater_c = s_set.heater_temperature;
    out.heater_ms = s_set.heater_duration;
    if (s_meas_mode == BME68X_SLEEP_MODE) {
        out.bsec_status = s_cycle_status;
        return 0;
    }
    struct bme68x_data d[3];
    uint8_t n = 0;
    int8_t r = BME68X_OK;
    if (s_meas_mode == BME68X_FORCED_MODE) {
        // The caller already waited the measurement time; allow a little slack.
        for (int i = 0; i < 10; i++) {
            r = bme68x_get_data(BME68X_FORCED_MODE, d, &n, &s_dev);
            if (r < 0) return r;
            if (n) break;
            ThisThread::sleep_for(10ms);
        }
        s_sensor_mode = BME68X_SLEEP_MODE;  // a forced measurement ends in sleep
        if (!n) return -100;                // no new data
    } else {
        r = bme68x_get_data(BME68X_PARALLEL_MODE, d, &n, &s_dev);
        if (r < 0) return r;
        if (!n) {
            out.bsec_status = s_cycle_status;
            return 0;  // nothing new yet this cycle
        }
    }

    const struct bme68x_data &last = d[n - 1];
    out.measured = out.ok = true;
    out.t_c = last.temperature;
    out.p_pa = last.pressure;
    out.rh = last.humidity;
    out.gas_run = s_set.run_gas || s_meas_mode == BME68X_PARALLEL_MODE;
    out.gas_valid = last.status & BME68X_GASM_VALID_MSK;
    out.heat_stab = last.status & BME68X_HEAT_STAB_MSK;
    out.gas_ohm = last.gas_resistance;
    out.res_heat = last.res_heat;

    if (s_bsec_ok) {
        // As Bosch's wrapper: only fields with a valid gas reading go to BSEC.
        for (uint8_t i = 0; i < n; i++)
            if (d[i].status & BME68X_GASM_VALID_MSK) run_bsec(d[i], out);
    }
    out.bsec_status = s_cycle_status;
    return 0;
}

bool maybe_save_state(int64_t now_ns, const Reading &r)
{
    static bool saved_at_acc3 = false;
    static int64_t last_save_ns = 0;  // boot counts as "just saved": the state was loaded then
    if (!s_bsec_ok || !r.bsec_out) return false;
    bool due = false;
    if (r.iaq_acc >= 3 && !saved_at_acc3) {
        due = true;  // first full calibration this boot
        saved_at_acc3 = true;
    } else if ((now_ns - last_save_ns) / 1000000000LL >= (int64_t)STATE_SAVE_PERIOD_S) {
        due = true;  // periodic, so a long run keeps its progress even below accuracy 3
    }
    if (!due) return false;
    last_save_ns = now_ns;
    return save_state();
}

}  // namespace bmebsec
