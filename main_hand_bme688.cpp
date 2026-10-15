#include "mbed.h"
#include "wdt.h"
#include "clkman.h"
#include "pwrman_regs.h"
#include "bess_proto.h"

// BESS firmware: WDT0 watchdog + BME688, TMP117 and ABP2 over I2C.
// BME688 follows Adafruit_BME680 / Bosch BME68x SensorAPI.
// Adafruit breakout labels: SCK = SCL -> P3_5, SDI = SDA -> P3_4 (I2CM1).
// CS/SDO are pulled up on the breakout, so the chip is in I2C mode at 0x77.
// Reads chip/variant ID, then forced-mode T/P/H/gas measurements.
// Older standalone tests (not built): main_wdt_test.cpp, bme688_full_test.cpp (SPI), abp2_only_test.cpp.
//
// minimal-printf has no float support, so values print as fixed-point ints.
//
// Each cycle also sends one machine-readable "$BESS,..." frame (protocol v1,
// see bess_proto.h) on the same serial port, for the gateway to log and score.

// RGB LED is open-drain, ACTIVE LOW. 0 = lit, 1 = off.
// Green = healthy heartbeat. Red = starved (stands in for the relay opening).
DigitalOut led_r(P2_4, 1);
DigitalOut led_g(P2_5, 1);

// Button reads 0 while pressed.
DigitalIn button(P2_3, PullUp);

I2C i2c(P3_4, P3_5);        // SDA, SCL

static const uint8_t REG_CHIP_ID   = 0xD0;  // expect 0x61
static const uint8_t REG_VARIANT   = 0xF0;  // 0x01 = BME688, 0x00 = BME680
static const uint8_t REG_RESET     = 0xE0;
static const uint8_t REG_CTRL_GAS0 = 0x70;
static const uint8_t REG_CTRL_GAS1 = 0x71;
static const uint8_t REG_CTRL_HUM  = 0x72;
static const uint8_t REG_CTRL_MEAS = 0x74;
static const uint8_t REG_CONFIG    = 0x75;
static const uint8_t REG_RES_HEAT0 = 0x5A;
static const uint8_t REG_GAS_WAIT0 = 0x64;
static const uint8_t REG_FIELD0    = 0x1D;

struct Calib {
    uint16_t t1; int16_t t2; int8_t t3;
    uint16_t p1; int16_t p2; int8_t p3; int16_t p4; int16_t p5;
    int8_t p6; int8_t p7; int16_t p8; int16_t p9; uint8_t p10;
    uint16_t h1; uint16_t h2; int8_t h3; int8_t h4; int8_t h5; uint8_t h6; int8_t h7;
    int8_t gh1; int16_t gh2; int8_t gh3;
    uint8_t res_heat_range; int8_t res_heat_val; int8_t range_sw_err;
};

static uint8_t variant;
static bess::Sample smp;      // this cycle's readings, sent as one frame
static bool wdt_boot = false; // this boot followed a watchdog reset
static Calib cal;
static int addr8;

static int reg_read(uint8_t reg, uint8_t *buf, int len)
{
    if (i2c.write(addr8, (const char *)&reg, 1, true) != 0) return -1;
    return i2c.read(addr8, (char *)buf, len) == 0 ? 0 : -2;
}

static int reg_write(uint8_t reg, uint8_t val)
{
    char b[2] = {(char)reg, (char)val};
    return i2c.write(addr8, b, 2) == 0 ? 0 : -1;
}

static int scan_found[16];
static int scan_count;

static void scan()
{
    printf("I2C1 scan:");
    scan_count = 0;
    for (int a = 0x08; a < 0x78; a++) {
        char dummy;
        if (i2c.read(a << 1, &dummy, 1) == 0) {
            printf(" 0x%02X", a);
            if (scan_count < 16) scan_found[scan_count++] = a;
        }
    }
    printf(scan_count ? "\r\n" : " (nothing)\r\n");
}

// Honeywell ABP2: write 0xAA 0x00 0x00 to start a conversion (~5 ms), then
// read status + 24-bit pressure + 24-bit temperature. Status: bit 6 = powered,
// bit 5 = busy, bit 2 = memory integrity error, bit 0 = math saturation.
// Pressure output is 10%..90% of 2^24 across the part's range.
static const uint32_t ABP2_OUT_MIN = 1677722;   // 10% of 2^24
static const uint32_t ABP2_OUT_MAX = 15099494;  // 90% of 2^24
// Part ABP2DANT001PG2A3XX: 0..1 psi gauge, I2C 0x28, 3.3 V.
static const float ABP2_PMIN_PSI = 0.0f;
static const float ABP2_PMAX_PSI = 1.0f;
static const float PA_PER_PSI = 6894.757f;
// Zero offset: the raw counts this sensor reads above OUT_MIN with its port open
// to air. Honeywell's +/-1.5 %FSS total error band for the 1 psi range assumes
// this auto-zero is applied (datasheet table 18, note 5). Measured 2026-10-15 on
// this unit: 22.73 Pa mean over 88 readings (sd 0.68 Pa) = 44241 counts.
// Re-measure after re-soldering, re-mounting or replacing the sensor, and
// occasionally in service (long-term drift up to 0.2 %FSS = 14 Pa per 1000 h).
// See "ABP2 zero offset" in SETUP.md. Set to 0 to send uncorrected readings.
static const int32_t ABP2_ZERO_COUNTS = 44241;
static int abp2_addr8;

static int abp2_read(uint8_t &status, uint32_t &p_counts, uint32_t &t_counts)
{
    const char cmd[3] = {(char)0xAA, 0x00, 0x00};
    if (i2c.write(abp2_addr8, cmd, 3) != 0) return -1;
    uint8_t d[7];
    for (int i = 0; i < 20; i++) {
        ThisThread::sleep_for(2ms);
        if (i2c.read(abp2_addr8, (char *)d, 1) != 0) return -2;
        if (!(d[0] & 0x20)) break;                  // not busy
        if (i == 19) return -3;
    }
    if (i2c.read(abp2_addr8, (char *)d, 7) != 0) return -2;
    status = d[0];
    p_counts = (uint32_t)d[1] << 16 | (uint32_t)d[2] << 8 | d[3];
    t_counts = (uint32_t)d[4] << 16 | (uint32_t)d[5] << 8 | d[6];
    if (!(status & 0x40)) return -4;                // not powered / not an ABP2
    if (status & 0x05) return -5;                   // memory error or saturation
    return 0;
}

static bool abp2_try(int a)
{
    abp2_addr8 = a << 1;
    uint8_t st = 0;
    uint32_t p = 0, t = 0;
    int r = abp2_read(st, p, t);
    if (r == -1 || r == -2)
        printf("  0x%02X: no ACK\r\n", a);
    else
        printf("  0x%02X: %s (err %d) status=0x%02X p_counts=%lu t_counts=%lu\r\n", a,
               r == 0 ? "ABP2 OK" : "responded but not a valid ABP2", r, st,
               (unsigned long)p, (unsigned long)t);
    return r == 0;
}

// Default address is 0x28; other parts ship at 0x08..0x78 steps of 0x10.
// Falls back to anything else the scan found.
static bool abp2_probe()
{
    if (abp2_try(0x28)) return true;
    for (int i = 0; i < scan_count; i++) {
        int a = scan_found[i];
        if (a == 0x28 || a == 0x48 || a == 0x77) continue;
        if (abp2_try(a)) return true;
    }
    return false;
}

// TMP117: 0x48-0x4B depending on ADD0. Reg 0x0F = device ID (0x0117),
// reg 0x00 = temperature, 7.8125 m°C/LSB. Powers up in continuous mode.
static I2C *tmp_bus;
static int tmp_addr8;

static int tmp_read16(uint8_t reg, uint16_t &v)
{
    uint8_t b[2];
    if (tmp_bus->write(tmp_addr8, (const char *)&reg, 1, true) != 0) return -1;
    if (tmp_bus->read(tmp_addr8, (char *)b, 2) != 0) return -2;
    v = (uint16_t)(b[0] << 8 | b[1]);
    return 0;
}

static bool tmp117_probe(I2C &bus, const char *name)
{
    tmp_bus = &bus;
    for (int a = 0x48; a <= 0x4B; a++) {
        tmp_addr8 = a << 1;
        uint16_t id;
        if (tmp_read16(0x0F, id) == 0) {
            printf("  %s 0x%02X: device_id=0x%04X %s\r\n", name, a, id,
                   (id & 0x0FFF) == 0x0117 ? "(TMP117)" : "(not TMP117)");
            if ((id & 0x0FFF) == 0x0117) return true;
        }
    }
    return false;
}

// Adafruit default is 0x77 (SDO high); 0x76 if SDO is tied low.
static uint8_t probe()
{
    uint8_t id = 0;
    for (int a : {0x77, 0x76}) {
        addr8 = a << 1;
        id = 0;
        int r = reg_read(REG_CHIP_ID, &id, 1);
        printf("  addr 0x%02X: %s chip_id=0x%02X\r\n", a, r == 0 ? "ACK" : "NACK", id);
        if (r == 0 && id == 0x61) return id;
    }
    return id;
}

static int load_calib()
{
    uint8_t c[42];
    if (reg_read(0x8A, c, 23)) return -1;
    if (reg_read(0xE1, c + 23, 14)) return -1;
    if (reg_read(0x00, c + 37, 5)) return -1;
    // Raw coefficient bytes in Bosch's coeff_array order (0x8A x23, 0xE1 x14,
    // 0x00 x5), so a host can recompute every reading with Bosch's own bme68x.c.
    printf("BME68x calib:");
    for (int i = 0; i < 42; i++) printf(" %02X", c[i]);
    printf("\r\n");

    cal.t1 = (uint16_t)(c[32] << 8 | c[31]);
    cal.t2 = (int16_t)(c[1] << 8 | c[0]);
    cal.t3 = (int8_t)c[2];
    cal.p1 = (uint16_t)(c[5] << 8 | c[4]);
    cal.p2 = (int16_t)(c[7] << 8 | c[6]);
    cal.p3 = (int8_t)c[8];
    cal.p4 = (int16_t)(c[11] << 8 | c[10]);
    cal.p5 = (int16_t)(c[13] << 8 | c[12]);
    cal.p7 = (int8_t)c[14];
    cal.p6 = (int8_t)c[15];
    cal.p8 = (int16_t)(c[19] << 8 | c[18]);
    cal.p9 = (int16_t)(c[21] << 8 | c[20]);
    cal.p10 = c[22];
    cal.h1 = (uint16_t)(c[25] << 4 | (c[24] & 0x0F));
    cal.h2 = (uint16_t)(c[23] << 4 | (c[24] >> 4));
    cal.h3 = (int8_t)c[26];
    cal.h4 = (int8_t)c[27];
    cal.h5 = (int8_t)c[28];
    cal.h6 = c[29];
    cal.h7 = (int8_t)c[30];
    cal.gh1 = (int8_t)c[35];
    cal.gh2 = (int16_t)(c[34] << 8 | c[33]);
    cal.gh3 = (int8_t)c[36];
    cal.res_heat_range = (c[39] & 0x30) >> 4;
    cal.res_heat_val = (int8_t)c[37];
    cal.range_sw_err = (int8_t)(c[41] & 0xF0) / 16;  // signed, as in Bosch get_calib_data
    return 0;
}

// Compensation formulas: Bosch BME68x SensorAPI, floating-point variant.
static float comp_temp(uint32_t adc, float &t_fine)
{
    float v1 = ((adc / 16384.0f) - (cal.t1 / 1024.0f)) * cal.t2;
    float d = (adc / 131072.0f) - (cal.t1 / 8192.0f);
    float v2 = d * d * (cal.t3 * 16.0f);
    t_fine = v1 + v2;
    return t_fine / 5120.0f;
}

static float comp_pres(uint32_t adc, float t_fine)
{
    float v1 = (t_fine / 2.0f) - 64000.0f;
    float v2 = v1 * v1 * (cal.p6 / 131072.0f);
    v2 = v2 + (v1 * cal.p5 * 2.0f);
    v2 = (v2 / 4.0f) + (cal.p4 * 65536.0f);
    v1 = (((cal.p3 * v1 * v1) / 16384.0f) + (cal.p2 * v1)) / 524288.0f;
    v1 = (1.0f + (v1 / 32768.0f)) * cal.p1;
    float p = 1048576.0f - adc;
    if (v1 == 0.0f) return 0.0f;
    p = ((p - (v2 / 4096.0f)) * 6250.0f) / v1;
    v1 = (cal.p9 * p * p) / 2147483648.0f;
    v2 = p * (cal.p8 / 32768.0f);
    float p256 = p / 256.0f;
    float v3 = p256 * p256 * p256 * (cal.p10 / 131072.0f);
    return p + (v1 + v2 + v3 + (cal.p7 * 128.0f)) / 16.0f;
}

static float comp_hum(uint16_t adc, float t_fine)
{
    float tc = t_fine / 5120.0f;
    float v1 = adc - ((cal.h1 * 16.0f) + ((cal.h3 / 2.0f) * tc));
    float v2 = v1 * ((cal.h2 / 262144.0f) *
                     (1.0f + ((cal.h4 / 16384.0f) * tc) + ((cal.h5 / 1048576.0f) * tc * tc)));
    float v3 = cal.h6 / 16384.0f;
    float v4 = cal.h7 / 2097152.0f;
    float h = v2 + ((v3 + (v4 * tc)) * v2 * v2);
    if (h > 100.0f) h = 100.0f;
    if (h < 0.0f) h = 0.0f;
    return h;
}

static uint8_t calc_res_heat(float target_c, float amb_c)
{
    if (target_c > 400.0f) target_c = 400.0f;
    float v1 = (cal.gh1 / 16.0f) + 49.0f;
    float v2 = ((cal.gh2 / 32768.0f) * 0.0005f) + 0.00235f;
    float v3 = cal.gh3 / 1024.0f;
    float v4 = v1 * (1.0f + (v2 * target_c));
    float v5 = v4 + (v3 * amb_c);
    return (uint8_t)(3.4f * ((v5 * (4.0f / (4.0f + cal.res_heat_range)) *
                              (1.0f / (1.0f + (cal.res_heat_val * 0.002f)))) - 25));
}

static uint8_t last_res_heat;  // heater set-point register value of the last gas run

static float comp_gas(uint16_t adc, uint8_t range)
{
    if (variant == 0x01) {  // BME688
        float v1 = (float)(262144u >> range);
        float v2 = 4096.0f + ((int32_t)adc - 512) * 3.0f;
        return 1000000.0f * v1 / v2;
    }
    static const float k1[16] = {0, 0, 0, 0, 0, -1, 0, -0.8f, 0, 0, -0.2f, -0.5f, 0, -1, 0, 0};
    static const float k2[16] = {0, 0, 0, 0, 0.1f, 0.7f, 0, -0.8f, -0.1f, 0, 0, 0, 0, 0, 0, 0};
    float v1 = 1340.0f + (5.0f * cal.range_sw_err);
    float v2 = v1 * (1.0f + k1[range] / 100.0f);
    float v3 = 1.0f + (k2[range] / 100.0f);
    return 1.0f / (v3 * 0.000000125f * (float)(1u << range) * (((adc - 512.0f) / v2) + 1.0f));
}

// Prints v with 2 decimals (minimal-printf has no %f).
static void print_fixed(const char *label, float v, const char *unit)
{
    int32_t x = (int32_t)(v * 100.0f + (v < 0 ? -0.5f : 0.5f));
    const char *sign = x < 0 ? "-" : "";
    if (x < 0) x = -x;
    printf("%s=%s%ld.%02ld%s", label, sign, (long)(x / 100), (long)(x % 100), unit);
}

static int measure(float amb_c, bool gas)
{
    reg_write(REG_CTRL_HUM, 0x02);                      // osrs_h x2
    reg_write(REG_CONFIG, 0x00);                        // filter off
    if (gas) {
        last_res_heat = calc_res_heat(320.0f, amb_c);
        reg_write(REG_RES_HEAT0, last_res_heat);
        reg_write(REG_GAS_WAIT0, 0x65);                 // 37 * 4 = 148 ms
        reg_write(REG_CTRL_GAS0, 0x00);                 // heater on
        reg_write(REG_CTRL_GAS1, variant == 0x01 ? 0x20 : 0x10);  // run_gas, profile 0
    } else {
        reg_write(REG_CTRL_GAS1, 0x00);
    }
    if (reg_write(REG_CTRL_MEAS, (0x2 << 5) | (0x5 << 2) | 0x1)) return -1;  // T x2, P x16, forced

    uint8_t d[17];
    for (int i = 0; i < 50; i++) {
        ThisThread::sleep_for(20ms);
        if (reg_read(REG_FIELD0, d, sizeof d)) return -2;
        if (d[0] & 0x80) break;                         // new_data
        if (i == 49) return -3;
    }

    uint32_t adc_p = (uint32_t)d[2] << 12 | (uint32_t)d[3] << 4 | d[4] >> 4;
    uint32_t adc_t = (uint32_t)d[5] << 12 | (uint32_t)d[6] << 4 | d[7] >> 4;
    uint16_t adc_h = (uint16_t)(d[8] << 8 | d[9]);
    uint8_t gs = variant == 0x01 ? d[16] : d[14];
    uint16_t adc_g = variant == 0x01 ? (uint16_t)(d[15] << 2 | d[16] >> 6)
                                     : (uint16_t)(d[13] << 2 | d[14] >> 6);

    float t_fine;
    float t = comp_temp(adc_t, t_fine);
    float p = comp_pres(adc_p, t_fine);
    float h = comp_hum(adc_h, t_fine);
    smp.bme_ok = true;
    smp.tbme_cC = bess::scaled(t, 100.0f);
    smp.p_Pa = bess::scaled(p, 1.0f);
    smp.rh_cP = bess::scaled(h, 100.0f);

    print_fixed("T", t, " C  ");
    print_fixed("P", p / 100.0f, " hPa  ");
    print_fixed("RH", h, " %");
    if (gas) {
        bool valid = gs & 0x20, stab = gs & 0x10;
        uint32_t gas_ohm = (uint32_t)comp_gas(adc_g, gs & 0x0F);
        printf("  gas=%lu ohm (valid=%d heat_stab=%d)", (unsigned long)gas_ohm, valid, stab);
        smp.gas_ok = true;
        smp.gas_ohm = gas_ohm;
        if (!valid || !stab) smp.flags |= bess::F_GAS_INVALID;
    }
    printf("  [raw T=%lu P=%lu H=%u", (unsigned long)adc_t, (unsigned long)adc_p, adc_h);
    if (gas) printf(" G=%u R=%u S=%02X HR=%u", adc_g, gs & 0x0F, gs & 0x30, last_res_heat);
    printf("]\r\n");
    return 0;
}

// Fires INT_PERIOD after the last feed. Both INT_PERIOD and RST_PERIOD
// count from the last feed (measured), so RST_PERIOD must be the longer one.
// Don't clear TIMEOUT here (that can cancel the reset); mask the IRQ instead.
extern "C" void WDT0_IRQHandler(void)
{
    led_g = 1;      // stop the heartbeat
    led_r = 0;      // where relay = OPEN will go
    NVIC_DisableIRQ(WDT0_IRQn);
}

// MAX14690 PMIC on the on-board I2C2 bus. Both 3.3 V LDOs are OFF at power-up.
// LDO2 feeds VDDB/VDDIOH and is enabled by low_level_init(). LDO3 feeds the
// header's 3V3 pin and is NOT enabled by mbed-os (Maxim's board library does
// it), so sensors powered from that pin get nothing until we turn it on here.
static I2C pmic_i2c(P5_7, P6_0);
static const int PMIC_ADDR8 = 0x28 << 1;
static const uint8_t PMIC_LDO2_CFG = 0x14;   // bits 2:1 = enable (01 = on)
static const uint8_t PMIC_LDO2_VSET = 0x15;  // 0.8 V + 0.1 V * code; 0x19 = 3.3 V
static const uint8_t PMIC_LDO3_CFG = 0x16;
static const uint8_t PMIC_LDO3_VSET = 0x17;

static int pmic_read(uint8_t reg, uint8_t &v)
{
    if (pmic_i2c.write(PMIC_ADDR8, (const char *)&reg, 1, true) != 0) return -1;
    return pmic_i2c.read(PMIC_ADDR8, (char *)&v, 1) == 0 ? 0 : -2;
}

static int pmic_write(uint8_t reg, uint8_t v)
{
    char b[2] = {(char)reg, (char)v};
    return pmic_i2c.write(PMIC_ADDR8, b, 2) == 0 ? 0 : -1;
}

// Reads an LDO's config, and enables it at 3.3 V (LDO mode) if it isn't already.
static void ldo_ensure_3v3(const char *name, uint8_t cfg_reg, uint8_t vset_reg)
{
    uint8_t cfg = 0, vset = 0;
    if (pmic_read(cfg_reg, cfg) || pmic_read(vset_reg, vset)) {
        printf("PMIC %s: read failed\r\n", name);
        return;
    }
    bool on = (cfg & 0x07) == 0x02;  // bits 2:1 = 01 enabled, bit 0 = 0 LDO mode
    printf("PMIC %s: cfg=0x%02X vset=0x%02X -> %s, setpoint %d mV\r\n", name, cfg, vset,
           on ? "ENABLED" : "OFF", 800 + 100 * (vset & 0x1F));
    if (!on || vset != 0x19) {
        pmic_write(vset_reg, 0x19);
        pmic_write(cfg_reg, 0x02);
        ThisThread::sleep_for(20ms);
        pmic_read(cfg_reg, cfg);
        pmic_read(vset_reg, vset);
        printf("PMIC %s: now enabled at 3.3 V -> cfg=0x%02X vset=0x%02X\r\n", name, cfg, vset);
    }
}

static void power_check()
{
    pmic_i2c.frequency(100000);
    ldo_ensure_3v3("LDO2 (VDDIOH/VDDB)", PMIC_LDO2_CFG, PMIC_LDO2_VSET);
    ldo_ensure_3v3("LDO3 (header 3V3 pin)", PMIC_LDO3_CFG, PMIC_LDO3_VSET);
    // Give sensors on the 3V3 pin time to power up (ABP2 needs 2.5 ms).
    ThisThread::sleep_for(50ms);
}

static void wdt_feed()
{
    WDT_Reset(MXC_WDT0);
}

static void wdt_setup()
{
    // WDT registers are cleared by the reset itself; the reset cause
    // survives in PWRMAN.
    uint32_t rst = MXC_PWRMAN->pwr_rst_ctrl;
    printf("pwr_rst_ctrl=0x%08lX\r\n", (unsigned long)rst);
    if (rst & MXC_F_PWRMAN_PWR_RST_CTRL_WATCHDOG_TIMEOUT) {
        printf("*** recovered from watchdog reset ***\r\n");
        wdt_boot = true;
    }

    sys_cfg_wdt_t cfg;
    cfg.clk = CLKMAN_WDT_SELECT_SCALED_SYS_CLK_CTRL;
    // WDT0 counts sysclk/scale regardless of what the header docs say;
    // CLKMAN_SCALE_DISABLED gates it off entirely (the watchdog never fires).
    // 96 MHz / 256 = 375 kHz.
    cfg.clk_scale = CLKMAN_SCALE_DIV_256;

    int r;
    r = WDT_Init(MXC_WDT0, &cfg, MXC_V_WDT_UNLOCK_KEY);
    printf("WDT_Init      -> %d\r\n", r);

    NVIC_SetVector(WDT0_IRQn, (uint32_t)WDT0_IRQHandler);
    NVIC_EnableIRQ(WDT0_IRQn);

    r = WDT_EnableInt(MXC_WDT0, WDT_PERIOD_2_20_CLKS, MXC_V_WDT_UNLOCK_KEY);
    printf("WDT_EnableInt -> %d\r\n", r);

    r = WDT_EnableReset(MXC_WDT0, WDT_PERIOD_2_21_CLKS, MXC_V_WDT_UNLOCK_KEY);
    printf("WDT_EnableRst -> %d\r\n", r);

    r = WDT_Start(MXC_WDT0, MXC_V_WDT_UNLOCK_KEY);
    wdt_feed();                   // feed immediately — boot printing is slow
    printf("WDT_Start     -> %d\r\n", r);
    printf("armed: warn ~2.8 s, reset ~5.6 s after last feed. press button to hang.\r\n");
    wdt_feed();
}

// Busy-wait so the RTOS can't run anything else. Prints elapsed time and
// flags so you can measure the real warn/reset periods.
static void hang(Timer &t)
{
    printf("HANGING at t=%llu ms — no more feeds\r\n",
           (unsigned long long)std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count());
    while (true) {
        printf("  hung t=%llu ms flags=0x%08lX\r\n",
               (unsigned long long)std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count(),
               (unsigned long)MXC_WDT0->flags);
        wait_us(500000);
    }
}

// Sensor cycle (~1.2 s worst case) + 2 s idle exceeds the 2.8 s warn period,
// so the idle time is sliced and fed. Also polls the hang button.
static void sleep_fed(Timer &t, int slices)
{
    for (int i = 0; i < slices; i++) {
        ThisThread::sleep_for(250ms);
        wdt_feed();
        if (button.read() == 0) hang(t);
    }
}

// Sensor missing: stop feeding and let the watchdog reset us (warn ~2.8 s,
// reset ~5.6 s). After reset the sensors are probed again.
static void fail(const char *msg)
{
    printf("FAIL: %s — no more feeds, watchdog will reset\r\n", msg);
    while (true) ThisThread::sleep_for(1s);
}

// Consecutive read failures before we treat a sensor as missing, so a single
// I2C glitch doesn't reset the board.
static const int MAX_READ_FAILS = 3;

int main()
{
    printf("\r\n=== BESS: watchdog + BME688 + TMP117 + ABP2 ===\r\n");
    wdt_setup();
    power_check();
    wdt_feed();

    Timer t;
    t.start();

    i2c.frequency(100000);
    // P3_4/P3_5 default to the 1.8 V VDDIO rail. The bare ABP2 (no level
    // shifter) only responds with the pins on VDDIOH (3.3 V): pin index
    // 28/29 -> use_vddioh_0 bits 28/29.
    MXC_IOMAN->use_vddioh_0 |= (1u << 28) | (1u << 29);
    ThisThread::sleep_for(10ms);
    scan();
    uint8_t id = probe();
    wdt_feed();
    if (id != 0x61) fail("no BME68x at 0x76/0x77 (expected chip_id 0x61)");

    printf("TMP117 probe:\r\n");
    bool have_tmp = tmp117_probe(i2c, "I2C1");
    wdt_feed();
    if (!have_tmp) fail("no TMP117 at 0x48-0x4B (expected device_id 0x0117)");

    printf("ABP2 probe:\r\n");
    if (!abp2_probe()) fail("no ABP2 found (status bit 6 never set)");
    wdt_feed();

    reg_read(REG_VARIANT, &variant, 1);
    printf("BME68x at 0x%02X, chip_id=0x61 variant=0x%02X (%s)\r\n", addr8 >> 1, variant,
           variant == 0x01 ? "BME688" : variant == 0x00 ? "BME680" : "unknown");

    reg_write(REG_RESET, 0xB6);
    ThisThread::sleep_for(10ms);
    if (load_calib()) fail("calibration read");
    wdt_feed();

    float amb = 25.0f;
    int tmp_fails = 0, bme_fails = 0, abp_fails = 0;
    for (int n = 0;; n++) {
        led_g = 0;
        wdt_feed();
        smp = bess::Sample();
        smp.seq = (uint32_t)n;
        smp.up_ms = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count();
        if (wdt_boot) smp.flags |= bess::F_WDT_BOOT;
        printf("#%d t=%llu ms  ", n,
               (unsigned long long)std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count());
        uint16_t raw;
        if (tmp_read16(0x00, raw) == 0) {
            float t117 = (int16_t)raw * 0.0078125f;
            print_fixed("T117", t117, " C | ");
            amb = t117;  // heater target is compensated for ambient
            tmp_fails = 0;
            smp.t117_ok = true;
            smp.t117_cC = bess::scaled(t117, 100.0f);
        } else {
            printf("T117 read error (%d/%d) | ", ++tmp_fails, MAX_READ_FAILS);
            smp.flags |= bess::F_TMP_FAIL;
        }
        uint8_t abp_st = 0;
        uint32_t abp_p = 0, abp_t = 0;
        int ar = abp2_read(abp_st, abp_p, abp_t);
        if (ar == 0) {
            // Zero-corrected: open port reads ~0 Pa. Can go slightly negative (noise).
            float psi = ((float)abp_p - ABP2_OUT_MIN - ABP2_ZERO_COUNTS) *
                        (ABP2_PMAX_PSI - ABP2_PMIN_PSI) / (ABP2_OUT_MAX - ABP2_OUT_MIN) + ABP2_PMIN_PSI;
            // psi printed as milli-psi to keep 5 significant digits with 2 decimals.
            print_fixed("ABP2 P", psi * 1000.0f, " mpsi");
            print_fixed(" (", psi * PA_PER_PSI, " Pa) ");
            float tabp = abp_t * 200.0f / 16777215.0f - 50.0f;
            print_fixed("T", tabp, " C | ");
            abp_fails = 0;
            smp.abp_ok = true;
            smp.dp_cPa = bess::scaled(psi * PA_PER_PSI, 100.0f);
            smp.tabp_cC = bess::scaled(tabp, 100.0f);
        } else {
            printf("ABP2 read error %d status=0x%02X (%d/%d) | ", ar, abp_st, ++abp_fails, MAX_READ_FAILS);
            smp.flags |= bess::F_ABP_FAIL;
        }
        int r = measure(amb, n > 0);  // first pass T/P/H only, then with gas heater
        if (r) {
            printf("BME688 measure error %d (%d/%d)\r\n", r, ++bme_fails, MAX_READ_FAILS);
            smp.bme_ok = false;
            smp.gas_ok = false;
            smp.flags |= bess::F_BME_FAIL;
        } else {
            bme_fails = 0;
        }
        // Sent before the fail checks: the last frame shows which sensor failed,
        // then the frames stop (the receiver's 6 s timeout catches the silence).
        char frame[bess::FRAME_MAX];
        if (bess::encode(smp, frame, sizeof frame)) printf("%s", frame);
        if (tmp_fails >= MAX_READ_FAILS) fail("TMP117 stopped responding");
        if (abp_fails >= MAX_READ_FAILS) fail("ABP2 stopped responding");
        if (bme_fails >= MAX_READ_FAILS) fail("BME688 stopped responding");
        led_g = 1;
        wdt_feed();
        sleep_fed(t, 8);  // 2 s
    }
}
