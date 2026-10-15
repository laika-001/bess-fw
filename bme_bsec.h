// BME688 through Bosch's BME68x Sensor API, driven by Bosch BSEC2 for the
// IAQ / CO2-equivalent / breath-VOC-equivalent estimates. BSEC's calibration
// state is kept in the last flash page so it survives resets and reflashes.
//
// One cycle, timed by BSEC (3 s in LP mode):
//   start()  asks BSEC for the sensor settings and starts the measurement,
//   ...      the caller reads its other sensors while the BME688 measures,
//   finish() reads the result and runs BSEC on it.
//
// If BSEC fails to start, the BME688 still runs: plain forced measurements at
// 320 C / 150 ms every 3 s, with the BSEC outputs left empty.
#pragma once
#include <stdint.h>

namespace mbed { class I2C; }

namespace bmebsec {

static const int64_t CYCLE_NS = 3000000000LL;  // BSEC LP sample interval (fallback uses the same)

struct Reading {
    bool measured = false;     // a measurement was taken this cycle
    bool ok = false;           // temperature/pressure/humidity are valid
    float t_c = 0, p_pa = 0, rh = 0;
    bool gas_run = false;      // the heater ran this cycle
    bool gas_valid = false, heat_stab = false;
    float gas_ohm = 0;
    uint16_t heater_c = 0, heater_ms = 0;
    uint8_t res_heat = 0;
    // BSEC outputs, present when bsec_out is true
    bool bsec_out = false;
    float iaq = 0, siaq = 0, co2eq = 0, bvoc = 0;
    uint8_t iaq_acc = 0;
    bool stab = false, run_in = false;
    int bsec_status = 0;       // last BSEC return code this cycle (0 = OK, <0 error, >0 warning)
};

// Brings up the BME688 at addr7 (Bosch init: soft reset, chip ID, calibration),
// then BSEC (config, saved state, subscription). Returns false only if the
// BME688 itself doesn't answer. Prints what it does.
bool begin(mbed::I2C &bus, uint8_t addr7, int64_t now_ns);

bool bsec_running();
uint8_t variant();                 // BME68x variant ID (1 = BME688)
int64_t next_call_ns();            // when start() should run next

// Starts this cycle's measurement. amb_c (TMP117) sets the heater's ambient
// compensation. Returns the time in microseconds until the data is ready
// (0: nothing to wait for), or a negative BME68x error code.
int32_t start(int64_t now_ns, float amb_c);

// Reads the measurement started by start() and runs BSEC on it.
// Returns 0, or a negative error code (BME688 read failed).
int finish(Reading &r);

// Saves BSEC's state to flash when it is worth it: the first time IAQ accuracy
// reaches 3 after boot, then every 6 hours. Returns true if it wrote.
bool maybe_save_state(int64_t now_ns, const Reading &r);

}  // namespace bmebsec
