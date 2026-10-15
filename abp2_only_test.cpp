#include "mbed.h"

// ABP2-only bring-up test (ABP2DANT001PG2A3XX: 0..1 psi gauge, I2C 0x28, 3.3 V).
// Wiring: pin 1 GND, pin 2 VDD (3.3 V), pin 5 SDA -> P3_4, pin 6 SCL -> P3_5.
// No watchdog: retries once a second so wiring can be adjusted live.
// Full firmware (watchdog + BME688 + TMP117 + ABP2) is main.cpp; this file is not built.

DigitalOut led_r(P2_4, 1);  // active low
DigitalOut led_g(P2_5, 1);

static const uint32_t ABP2_OUT_MIN = 1677722;   // 10% of 2^24
static const uint32_t ABP2_OUT_MAX = 15099494;  // 90% of 2^24
static const float ABP2_PMAX_PSI = 1.0f;
static const float PA_PER_PSI = 6894.757f;
static const int ABP2_ADDR8 = 0x28 << 1;

// Prints v with 2 decimals (minimal-printf has no %f).
static void print_fixed(const char *label, float v, const char *unit)
{
    int32_t x = (int32_t)(v * 100.0f + (v < 0 ? -0.5f : 0.5f));
    const char *sign = x < 0 ? "-" : "";
    if (x < 0) x = -x;
    printf("%s=%s%ld.%02ld%s", label, sign, (long)(x / 100), (long)(x % 100), unit);
}

int main()
{
    printf("\r\n=== ABP2-only test (SDA=P3_4 SCL=P3_5, addr 0x28) ===\r\n");

    // With a weak internal pull-down, a line only reads 1 if an external
    // pull-up is really holding it high.
    {
        DigitalIn sda(P3_4, PullDown), scl(P3_5, PullDown);
        wait_us(2000);
        printf("idle line levels: SDA=%d SCL=%d (need 1 and 1)\r\n", sda.read(), scl.read());
    }

    I2C i2c(P3_4, P3_5);
    i2c.frequency(100000);

    // P3_4/P3_5 default to the 1.8 V VDDIO rail. The bare ABP2 (no level
    // shifter) needs a logic high near its 3.3 V supply, so after a few tries
    // move both pins to VDDIOH (3.3 V): pin index 28/29 -> use_vddioh_0 bits 28/29.
    const uint32_t i2c1_mask = (1u << 28) | (1u << 29);
    for (int n = 0;; n++) {
        if (n == 3) {
            MXC_IOMAN->use_vddioh_0 |= i2c1_mask;
            printf("-- switched P3_4/P3_5 to VDDIOH (3.3 V I/O) --\r\n");
            ThisThread::sleep_for(10ms);
        }
        printf("#%d [%s] scan:", n, (MXC_IOMAN->use_vddioh_0 & i2c1_mask) == i2c1_mask ? "3.3V" : "1.8V");
        int found = 0;
        for (int a = 0x08; a < 0x78; a++) {
            char dummy;
            if (i2c.read(a << 1, &dummy, 1) == 0) { printf(" 0x%02X", a); found++; }
        }
        if (!found) printf(" (nothing)");

        // Measure command, wait for busy (status bit 5) to clear, read 7 bytes.
        const char cmd[3] = {(char)0xAA, 0x00, 0x00};
        uint8_t d[7] = {0};
        if (i2c.write(ABP2_ADDR8, cmd, 3) != 0) {
            printf(" | ABP2: no ACK to measure command\r\n");
            led_g = 1; led_r = 0;
        } else {
            ThisThread::sleep_for(10ms);
            if (i2c.read(ABP2_ADDR8, (char *)d, 7) != 0) {
                printf(" | ABP2: ACKed command but read failed\r\n");
            } else {
                uint32_t p = (uint32_t)d[1] << 16 | (uint32_t)d[2] << 8 | d[3];
                uint32_t t = (uint32_t)d[4] << 16 | (uint32_t)d[5] << 8 | d[6];
                float psi = ((float)p - ABP2_OUT_MIN) * ABP2_PMAX_PSI / (ABP2_OUT_MAX - ABP2_OUT_MIN);
                printf(" | ABP2 status=0x%02X%s ", d[0],
                       (d[0] & 0x40) ? "" : " (power bit clear!)");
                print_fixed("P", psi * 1000.0f, " mpsi");
                print_fixed(" (", psi * PA_PER_PSI, " Pa)");
                print_fixed(" T", t * 200.0f / 16777215.0f - 50.0f, " C");
                printf(" [raw p=%lu t=%lu]\r\n", (unsigned long)p, (unsigned long)t);
                led_r = 1; led_g = 0;
            }
        }
        ThisThread::sleep_for(1s);
        led_g = 1;
    }
}
