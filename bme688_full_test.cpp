#include "mbed.h"

// BME688 bring-up test over bit-banged SPI (mode 0). The sensor is wired to
// pins with no hardware SPI function: SCK=P3_5, SDI=P3_4, SDO=P5_2, CS=P3_0.
// Checks chip/variant ID, then runs forced-mode T/P/H/gas measurements.
// Original watchdog test lives in main_wdt_test.cpp (not built).
//
// minimal-printf has no float support, so values print as fixed-point ints.

DigitalOut led_r(P2_4, 1);  // active low
DigitalOut led_g(P2_5, 1);

DigitalOut sck(P3_5, 0);    // mode 0: idle low
DigitalOut mosi(P3_4, 0);   // -> BME688 SDI
DigitalIn miso(P5_2);       // <- BME688 SDO
DigitalOut cs(P3_0, 1);

// MSB first; sensor samples SDI and we sample SDO on the rising edge.
static uint8_t spi_xfer(uint8_t out)
{
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        mosi = (out >> i) & 1;
        wait_us(1);
        sck = 1;
        in = (uint8_t)(in << 1 | miso.read());
        wait_us(1);
        sck = 0;
    }
    return in;
}

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

static const uint8_t REG_STATUS = 0x73;  // spi_mem_page is bit 4, visible on both pages

static uint8_t variant;
static Calib cal;
static int cur_page = -1;

// SPI: bit 7 set = read. Registers 0x80-0xFF are page 0 (spi_mem_page=0),
// 0x00-0x7F are page 1 (spi_mem_page=1); the 7-bit SPI address is reg & 0x7F.
static void raw_read(uint8_t reg7, uint8_t *buf, int len)
{
    cs = 0;
    spi_xfer(reg7 | 0x80);
    for (int i = 0; i < len; i++) buf[i] = (uint8_t)spi_xfer(0x00);
    cs = 1;
}

static void raw_write(uint8_t reg7, uint8_t val)
{
    cs = 0;
    spi_xfer(reg7 & 0x7F);
    spi_xfer(val);
    cs = 1;
}

static void set_page(uint8_t reg)
{
    int want = reg > 0x7F ? 0 : 1;
    if (want == cur_page) return;
    uint8_t st;
    raw_read(REG_STATUS, &st, 1);
    raw_write(REG_STATUS, (st & ~0x10) | (want << 4));
    cur_page = want;
}

static int reg_read(uint8_t reg, uint8_t *buf, int len)
{
    set_page(reg);
    raw_read(reg & 0x7F, buf, len);
    return 0;
}

static int reg_write(uint8_t reg, uint8_t val)
{
    set_page(reg);
    raw_write(reg & 0x7F, val);
    return 0;
}

// A CSB falling edge after power-up latches the sensor into SPI mode.
static void select_spi_mode()
{
    cs = 0; wait_us(1); cs = 1; wait_us(1);
    cur_page = -1;
}

static uint8_t read_chip_id()
{
    select_spi_mode();
    uint8_t id = 0;
    reg_read(REG_CHIP_ID, &id, 1);
    return id;
}

static int load_calib()
{
    uint8_t c[42];
    if (reg_read(0x8A, c, 23)) return -1;
    if (reg_read(0xE1, c + 23, 14)) return -1;
    if (reg_read(0x00, c + 37, 5)) return -1;

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
    cal.range_sw_err = ((int8_t)c[41] & 0xF0) >> 4;
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
        reg_write(REG_RES_HEAT0, calc_res_heat(320.0f, amb_c));
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

    print_fixed("T", t, " C  ");
    print_fixed("P", p / 100.0f, " hPa  ");
    print_fixed("RH", h, " %");
    if (gas) {
        bool valid = gs & 0x20, stab = gs & 0x10;
        printf("  gas=%lu ohm (valid=%d heat_stab=%d)",
               (unsigned long)comp_gas(adc_g, gs & 0x0F), valid, stab);
    }
    printf("  [raw T=%lu P=%lu H=%u]\r\n", (unsigned long)adc_t, (unsigned long)adc_p, adc_h);
    return 0;
}

static void fail(const char *msg)
{
    printf("FAIL: %s\r\n", msg);
    while (true) { led_r = !led_r; ThisThread::sleep_for(250ms); }
}

int main()
{
    printf("\r\n=== BME688 bit-bang SPI test (SCK=P3_5 SDI=P3_4 SDO=P5_2 CS=P3_0) ===\r\n");

    // Pin index = port*8 + pin: P3_0/P3_4/P3_5 -> use_vddioh_0 bits 24/28/29,
    // P5_2 -> use_vddioh_1 bit 10.
    const uint32_t vddioh0_mask = (1u << 24) | (1u << 28) | (1u << 29);
    const uint32_t vddioh1_mask = 1u << 10;
    printf("MISO idle level (CS high) = %d\r\n", miso.read());

    uint8_t id = read_chip_id();
    printf("chip_id=0x%02X at default 1.8V I/O\r\n", id);
    if (id != 0x61) {
        MXC_IOMAN->use_vddioh_0 |= vddioh0_mask;
        MXC_IOMAN->use_vddioh_1 |= vddioh1_mask;
        ThisThread::sleep_for(5ms);
        id = read_chip_id();
        printf("chip_id=0x%02X with pins on VDDIOH (3.3V)\r\n", id);
    }
    if (id != 0x61) {
        fail(id == 0x00 ? "SDO reads 0x00 (expected 0x61)"
           : id == 0xFF ? "SDO reads 0xFF (expected 0x61)"
           : "unexpected chip id (expected 0x61)");
    }

    reg_read(REG_VARIANT, &variant, 1);
    printf("chip_id=0x61 variant=0x%02X (%s)\r\n", variant,
           variant == 0x01 ? "BME688" : variant == 0x00 ? "BME680" : "unknown");

    reg_write(REG_RESET, 0xB6);
    ThisThread::sleep_for(10ms);
    if (read_chip_id() != 0x61) fail("lost sensor after soft reset");
    if (load_calib()) fail("calibration read");
    printf("calib: t1=%u t2=%d t3=%d p1=%u h1=%u h2=%u gh1=%d gh2=%d gh3=%d\r\n",
           cal.t1, cal.t2, cal.t3, cal.p1, cal.h1, cal.h2, cal.gh1, cal.gh2, cal.gh3);

    float amb = 25.0f;
    for (int n = 0;; n++) {
        led_g = 0;
        printf("#%d ", n);
        int r = measure(amb, n > 0);  // first pass T/P/H only, then with gas heater
        if (r) printf("measure error %d\r\n", r);
        led_g = 1;
        ThisThread::sleep_for(2s);
    }
}
