#include "mbed.h"
#include "wdt.h"
#include "clkman.h"
#include "pwrman_regs.h"

// RGB LED is open-drain, ACTIVE LOW. 0 = lit, 1 = off.
// Green = healthy heartbeat. Red = starved (stands in for the relay opening).
DigitalOut led_r(P2_4, 1);
DigitalOut led_g(P2_5, 1);

// Button reads 0 while pressed.
DigitalIn button(P2_3, PullUp);

// Fires INT_PERIOD after the last feed. Both INT_PERIOD and RST_PERIOD
// count from the last feed (measured), so RST_PERIOD must be the longer one.
// Don't clear TIMEOUT here (that can cancel the reset); mask the IRQ instead.
extern "C" void WDT0_IRQHandler(void)
{
    led_g = 1;      // stop the heartbeat
    led_r = 0;      // where relay = OPEN will go
    NVIC_DisableIRQ(WDT0_IRQn);
}

int main()
{
    printf("\r\n=== BESS watchdog test ===\r\n");

    // WDT registers are cleared by the reset itself; the reset cause
    // survives in PWRMAN.
    uint32_t rst = MXC_PWRMAN->pwr_rst_ctrl;
    printf("pwr_rst_ctrl=0x%08lX\r\n", (unsigned long)rst);
    if (rst & MXC_F_PWRMAN_PWR_RST_CTRL_WATCHDOG_TIMEOUT) {
        printf("*** recovered from watchdog reset ***\r\n");
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
    WDT_Reset(MXC_WDT0);          // feed immediately — boot printing is slow
    printf("WDT_Start     -> %d\r\n", r);

    WDT_Reset(MXC_WDT0);
    printf("ctrl=0x%08lX enable=0x%08lX flags=0x%08lX\r\n",
           (unsigned long)MXC_WDT0->ctrl,
           (unsigned long)MXC_WDT0->enable,
           (unsigned long)MXC_WDT0->flags);

    WDT_Reset(MXC_WDT0);
    printf("armed: warn ~2.8 s, reset ~5.6 s after last feed. press button to hang.\r\n");

    Timer t;
    t.start();
    int tick = 0;

    while (true) {
        led_g = 0;
        ThisThread::sleep_for(50ms);
        led_g = 1;

        WDT_Reset(MXC_WDT0);
        printf("tick %d  (t=%llu ms)\r\n", tick++,
               (unsigned long long)std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count());

        if (button.read() == 0) {
            printf("HANGING at t=%llu ms — no more feeds\r\n",
                   (unsigned long long)std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count());

            // Busy-wait so the RTOS can't run anything else. Prints elapsed
            // time and flags so you can measure the real warn/reset periods.
            while (true) {
                printf("  hung t=%llu ms flags=0x%08lX\r\n",
                       (unsigned long long)std::chrono::duration_cast<std::chrono::milliseconds>(t.elapsed_time()).count(),
                       (unsigned long)MXC_WDT0->flags);
                wait_us(500000);
            }
        }

        ThisThread::sleep_for(1s);
    }
}