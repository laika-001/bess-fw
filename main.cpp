#include "mbed.h"
#include "wdt.h"
#include "clkman.h"
#include "pwrman_regs.h"
#include "bess_proto.h"
#include "bme_bsec.h"
#include "rules.h"

// BESS firmware: WDT0 watchdog + BME688, TMP117 and ABP2 over I2C.
// BME688: Bosch BME68x Sensor API + Bosch BSEC2 (IAQ, CO2-eq, bVOC-eq), see bme_bsec.cpp.
// Adafruit breakout labels: SCK = SCL -> P3_5, SDI = SDA -> P3_4 (I2CM1).
// CS/SDO are pulled up on the breakout, so the chip is in I2C mode at 0x77.
// The cycle is timed by BSEC: one measurement every 3 s (LP mode).
// Older standalone tests (not built): main_wdt_test.cpp, bme688_full_test.cpp (SPI), abp2_only_test.cpp.
// main_sensors_v1.cpp: before the data frames; main_hand_bme688.cpp: before BSEC.
//
// minimal-printf has no float support, so values print as fixed-point ints.
//
// Each cycle also sends one machine-readable "$BESS,..." frame (protocol v4,
// see bess_proto.h) on the same serial port and to the Pi on UART2.
// Threshold checks run here on the module (rules.h, see DETECTION.md): the
// per-channel results go in the frame, and F_LOCAL_ALARM is set when two
// channels trip within a minute (possible battery event).

// RGB LED is open-drain, ACTIVE LOW. 0 = lit, 1 = off.
// Green = healthy heartbeat. Red = starved (stands in for the relay opening).
DigitalOut led_r(P2_4, 1);
DigitalOut led_g(P2_5, 1);

// Button reads 0 while pressed.
DigitalIn button(P2_3, PullUp);

I2C i2c(P3_4, P3_5);        // SDA, SCL

// Raspberry Pi gateway link: UART2, 115200 8N1, $BESS frames only (no debug
// text). MAX P3_1 = TX -> Pi pin 10 (GPIO15 RXD), MAX P3_0 = RX <- Pi pin 8
// (GPIO14 TXD), plus GND. If the wires are crossed the other way, set
// PI_UART_SWAP 1 (UART2 can map either pin to TX).
#ifndef PI_UART_SWAP
#define PI_UART_SWAP 0
#endif
static BufferedSerial pi_uart(PI_UART_SWAP ? P3_0 : P3_1, PI_UART_SWAP ? P3_1 : P3_0, 115200);

static const uint8_t REG_CHIP_ID   = 0xD0;  // expect 0x61
static bess::Sample smp;      // this cycle's readings, sent as one frame
static rules::Checker checker;  // on-board threshold checks
static bool wdt_boot = false; // this boot followed a watchdog reset
static int addr8;             // BME68x address found by probe()

static int reg_read(uint8_t reg, uint8_t *buf, int len)
{
    if (i2c.write(addr8, (const char *)&reg, 1, true) != 0) return -1;
    return i2c.read(addr8, (char *)buf, len) == 0 ? 0 : -2;
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

// Prints v with 2 decimals (minimal-printf has no %f).
static void print_fixed(const char *label, float v, const char *unit)
{
    int32_t x = (int32_t)(v * 100.0f + (v < 0 ? -0.5f : 0.5f));
    const char *sign = x < 0 ? "-" : "";
    if (x < 0) x = -x;
    printf("%s=%s%ld.%02ld%s", label, sign, (long)(x / 100), (long)(x % 100), unit);
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

static int64_t now_ns(Timer &t)
{
    return (int64_t)std::chrono::duration_cast<std::chrono::microseconds>(t.elapsed_time()).count() * 1000;
}

// Sleeps until the clock reaches target_ns, in slices short enough to keep
// feeding the watchdog (warn period 2.8 s). Also polls the hang button.
static void sleep_until_fed(Timer &t, int64_t target_ns)
{
    while (true) {
        int64_t left_ms = (target_ns - now_ns(t) + 999999) / 1000000;
        if (left_ms <= 0) return;
        if (left_ms > 250) left_ms = 250;
        ThisThread::sleep_for(std::chrono::milliseconds(left_ms));
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
    printf("\r\n=== BESS: watchdog + BME688/BSEC2 + TMP117 + ABP2 ===\r\n");
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
    // P3_0/P3_1 (Pi UART) on VDDIOH too: the Pi's GPIO are 3.3 V, and a 1.8 V
    // high from the MAX is below the Pi's input threshold. Pin index 24/25.
    MXC_IOMAN->use_vddioh_0 |= (1u << 24) | (1u << 25);
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

    if (!bmebsec::begin(i2c, (uint8_t)(addr8 >> 1), now_ns(t))) fail("BME68x init (Bosch API)");
    wdt_feed();

    float amb = 25.0f;  // TMP117 air temperature, for the BME688 heater's ambient compensation
    uint16_t raw;
    if (tmp_read16(0x00, raw) == 0) amb = (int16_t)raw * 0.0078125f;

    int tmp_fails = 0, bme_fails = 0, abp_fails = 0;
    for (int n = 0;; n++) {
        sleep_until_fed(t, bmebsec::next_call_ns());
        int64_t t0 = now_ns(t);
        led_g = 0;
        wdt_feed();
        smp = bess::Sample();
        smp.seq = (uint32_t)n;
        smp.up_ms = (uint32_t)(t0 / 1000000);
        if (wdt_boot) smp.flags |= bess::F_WDT_BOOT;

        // Start the BME688 first; read the other sensors while its heater runs.
        int32_t meas_us = bmebsec::start(t0, amb);
        int64_t ready_ns = t0 + (int64_t)(meas_us > 0 ? meas_us : 0) * 1000;

        printf("#%d t=%llu ms  ", n, (unsigned long long)(t0 / 1000000));
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

        bmebsec::Reading br;
        int r = meas_us < 0 ? (int)meas_us : 0;
        if (r == 0) {
            sleep_until_fed(t, ready_ns);
            r = bmebsec::finish(br);
        }
        if (r) {
            printf("BME688 error %d (%d/%d)", r, ++bme_fails, MAX_READ_FAILS);
            smp.flags |= bess::F_BME_FAIL;
        } else if (br.measured) {
            bme_fails = 0;
            smp.bme_ok = true;
            smp.tbme_cC = bess::scaled(br.t_c, 100.0f);
            smp.p_Pa = bess::scaled(br.p_pa, 1.0f);
            smp.rh_cP = bess::scaled(br.rh, 100.0f);
            print_fixed("T", br.t_c, " C  ");
            print_fixed("P", br.p_pa / 100.0f, " hPa  ");
            print_fixed("RH", br.rh, " %");
            if (br.gas_run) {
                smp.gas_ok = true;
                smp.gas_ohm = (uint32_t)(br.gas_ohm + 0.5f);
                smp.heat_c = br.heater_c;
                printf("  gas=%lu ohm (valid=%d heat_stab=%d)", (unsigned long)smp.gas_ohm, br.gas_valid, br.heat_stab);
                if (!br.gas_valid || !br.heat_stab) smp.flags |= bess::F_GAS_INVALID;
            }
            if (br.bsec_out) {
                smp.bsec_ok = true;
                smp.iaq_c = bess::scaled(br.iaq, 100.0f);
                smp.iaq_acc = br.iaq_acc;
                smp.siaq_c = bess::scaled(br.siaq, 100.0f);
                smp.co2eq_cppm = bess::scaled(br.co2eq, 100.0f);
                smp.bvoc_ppb = bess::scaled(br.bvoc, 1000.0f);
                smp.bsec_st = (br.stab ? 1 : 0) | (br.run_in ? 2 : 0);
                print_fixed(" | IAQ", br.iaq, "");
                printf(" (acc %u)", br.iaq_acc);
                print_fixed(" sIAQ", br.siaq, "");
                print_fixed(" CO2eq", br.co2eq, " ppm");
                print_fixed(" bVOCeq", br.bvoc, " ppm");
                printf(" stab=%d run_in=%d", br.stab, br.run_in);
            }
        } else {
            printf("BME688: no measurement this cycle");
        }
        if (!bmebsec::bsec_running() || br.bsec_status < 0) smp.flags |= bess::F_BSEC_ERR;
        if (br.bsec_status) printf("  [BSEC status %d]", br.bsec_status);
        printf("  [heater %u C %u ms HR=%u]\r\n", br.heater_c, br.heater_ms, br.res_heat);

        // On-board threshold checks, on exactly the values this frame carries.
        {
            rules::Input ri;
            ri.seq = smp.seq;
            ri.up_ms = smp.up_ms;
            ri.t_ok = smp.t117_ok; ri.t117 = smp.t117_cC / 100.0;
            ri.dp_ok = smp.abp_ok; ri.dp = smp.dp_cPa / 100.0;
            ri.gas_ok = smp.gas_ok; ri.gas = smp.gas_ohm; ri.heat_c = smp.heat_c;
            ri.gas_invalid = smp.flags & bess::F_GAS_INVALID;
            smp.rules = checker.update(ri);
            if (smp.rules & rules::EVENT) smp.flags |= bess::F_LOCAL_ALARM;
            if (smp.rules) {
                printf("  rules 0x%04X:%s%s%s%s%s\r\n", smp.rules,
                       smp.rules & rules::EVENT ? " POSSIBLE BATTERY EVENT" : "",
                       smp.rules & rules::GAS_WARN ? " gas" : "", smp.rules & rules::DP_WARN ? " pressure" : "",
                       smp.rules & (rules::T_WARN | rules::T_RISE) ? " temperature" : "",
                       smp.rules & rules::GAS_DRIFT ? " gas-drift" : "");
            }
        }
        // Sent before the fail checks: the last frame shows which sensor failed,
        // then the frames stop (the receiver's 6 s timeout catches the silence).
        char frame[bess::FRAME_MAX];
        size_t flen = bess::encode(smp, frame, sizeof frame);
        if (flen) {
            printf("%s", frame);
            pi_uart.write(frame, flen);  // buffered: returns at once, sends in the background
        }
        wdt_feed();
        bmebsec::maybe_save_state(now_ns(t), br);
        if (tmp_fails >= MAX_READ_FAILS) fail("TMP117 stopped responding");
        if (abp_fails >= MAX_READ_FAILS) fail("ABP2 stopped responding");
        if (bme_fails >= MAX_READ_FAILS) fail("BME688 stopped responding");
        led_g = 1;
        wdt_feed();
    }
}
