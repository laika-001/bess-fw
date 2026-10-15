// On-board threshold checks for battery thermal-runaway signatures.
// No mbed dependencies: the same code is replayed over logged frames on a PC
// (tools/check_rules.py). The thresholds here are the single source: the Pi
// dashboard reads them from this file at deploy time (pi/deploy.sh).
//
// Three channels, each judged every frame (see DETECTION.md for the signatures):
//   gas          BME688 gas resistance at the 320 C heater step, as a ratio to its
//                clean-air level. Venting electrolyte / H2 / CO lower it.
//   pressure     ABP2 enclosure gauge pressure. A venting cell gives a pulse.
//   temperature  TMP117 level, and its rate of rise over the last 2 minutes.
//
// One channel alone is usually an everyday disturbance (gas alone: people or
// solvents; pressure alone: a door or HVAC; slow temperature: sun). So the
// battery-event alarm (frame flag F_LOCAL_ALARM) needs two channels abnormal
// within the same minute, or the temperature at its critical level.
// A slow, persistent gas drift raises a separate maintenance warning
// (possible electrolyte leak or cracked seal).
#pragma once
#include <math.h>
#include <stdint.h>

namespace rules {

// ---- thresholds -------------------------------------------------------------
// GAS_REF_OHM: median gas resistance on the healthy night of 2026-10-15
// (02:00-07:30, 6599 readings, range 54.5-66.7 kOhm). Re-measure in clean air
// after replacing or re-mounting the BME688.
static const double GAS_REF_OHM = 63100.0;
static const double GAS_WARN_RATIO = 0.60;          // gas <= 60 % of clean-air level
static const double GAS_CRIT_RATIO = 0.35;          // gas <= 35 %
static const double GAS_DRIFT_RATIO = 0.80;         // gas <= 80 % continuously ...
static const uint32_t GAS_DRIFT_MS = 3600000;       // ... for an hour: leak warning
static const double DP_WARN_PA = 50.0;              // |enclosure gauge pressure|
static const double DP_CRIT_PA = 200.0;
static const double T_WARN_C = 45.0;                // TMP117 air temperature
static const double T_CRIT_C = 60.0;
static const double T_RISE_WARN_C_PER_MIN = 1.0;    // TMP117 rise ...
static const uint32_t T_RISE_WINDOW_MS = 120000;    // ... over the last 2 minutes
static const uint32_t COINCIDENCE_MS = 60000;       // channels "together" = within 1 minute

// ---- result bits (frame field "rules", 4 hex digits) -------------------------
enum Bit : uint16_t {
    GAS_WARN  = 0x0001,
    GAS_CRIT  = 0x0002,
    DP_WARN   = 0x0004,
    DP_CRIT   = 0x0008,
    T_WARN    = 0x0010,
    T_CRIT    = 0x0020,
    T_RISE    = 0x0040,
    GAS_DRIFT = 0x0080,   // slow persistent gas drop: maintenance warning
    EVENT     = 0x0100,   // possible battery event: >= 2 channels within a minute, or T_CRIT
};

struct Input {            // one frame's values, as sent in the frame
    uint32_t seq, up_ms;
    bool t_ok; double t117;       // degC
    bool dp_ok; double dp;        // Pa
    bool gas_ok; double gas;      // ohm
    uint16_t heat_c;
    bool gas_invalid;             // frame flag F_GAS_INVALID
};

class Checker {
public:
    double gas_ratio = -1;        // last valid gas / GAS_REF_OHM (-1 = none yet)
    double t_rate = 0;            // degC/min over the last 2 min
    bool t_rate_ok = false;

    // Returns the result bits for this frame.
    uint16_t update(const Input &in) {
        uint32_t ms = in.up_ms;
        if (have_last_ && (ms - last_ms_ > 60000 || in.seq <= last_seq_)) reset();  // reboot or gap
        have_last_ = true; last_ms_ = ms; last_seq_ = in.seq;
        uint16_t out = 0;

        // gas: only the steady 320 C heater step is comparable to the reference
        if (in.gas_ok && in.gas > 0 && in.heat_c == 320 && !in.gas_invalid) {
            gas_ratio = in.gas / GAS_REF_OHM;
            if (gas_ratio <= GAS_DRIFT_RATIO) { if (!drift_on_) { drift_on_ = true; drift_since_ = ms; } }
            else drift_on_ = false;
        }
        if (gas_ratio >= 0) {
            if (gas_ratio <= GAS_CRIT_RATIO) out |= GAS_CRIT | GAS_WARN;
            else if (gas_ratio <= GAS_WARN_RATIO) out |= GAS_WARN;
        }
        if (drift_on_ && ms - drift_since_ >= GAS_DRIFT_MS) out |= GAS_DRIFT;

        // enclosure pressure
        if (in.dp_ok) {
            double a = fabs(in.dp);
            if (a >= DP_CRIT_PA) out |= DP_CRIT | DP_WARN;
            else if (a >= DP_WARN_PA) out |= DP_WARN;
        }

        // temperature level and rate of rise
        if (in.t_ok) {
            if (in.t117 >= T_CRIT_C) out |= T_CRIT | T_WARN;
            else if (in.t117 >= T_WARN_C) out |= T_WARN;
            push_t(ms, in.t117);
            t_rate_ok = slope(ms, t_rate);
            if (t_rate_ok && t_rate >= T_RISE_WARN_C_PER_MIN) out |= T_RISE;
        }

        // coincidence: each channel counts as active for a minute after it last tripped
        if (out & GAS_WARN) { gas_seen_ = true; gas_ms_ = ms; }
        if (out & DP_WARN) { dp_seen_ = true; dp_ms_ = ms; }
        if (out & (T_WARN | T_RISE)) { t_seen_ = true; t_ms_ = ms; }
        int n = (gas_seen_ && ms - gas_ms_ <= COINCIDENCE_MS) + (dp_seen_ && ms - dp_ms_ <= COINCIDENCE_MS) +
                (t_seen_ && ms - t_ms_ <= COINCIDENCE_MS);
        if (n >= 2 || (out & T_CRIT)) out |= EVENT;
        return out;
    }

private:
    static const int N = 64;      // 2 min at one frame per 3 s is ~40
    uint32_t tt_[N]; double tv_[N];
    int head_ = 0, n_ = 0;
    bool have_last_ = false;
    uint32_t last_ms_ = 0, last_seq_ = 0;
    bool drift_on_ = false; uint32_t drift_since_ = 0;
    bool gas_seen_ = false, dp_seen_ = false, t_seen_ = false;
    uint32_t gas_ms_ = 0, dp_ms_ = 0, t_ms_ = 0;

    void reset() {
        head_ = n_ = 0;
        gas_ratio = -1; t_rate = 0; t_rate_ok = false;
        drift_on_ = false;
        gas_seen_ = dp_seen_ = t_seen_ = false;
    }
    void push_t(uint32_t ms, double v) {
        int i = (head_ + n_) % N;
        if (n_ == N) head_ = (head_ + 1) % N; else n_++;
        tt_[i] = ms; tv_[i] = v;
        while (n_ && ms - tt_[head_] > T_RISE_WINDOW_MS) { head_ = (head_ + 1) % N; n_--; }
    }
    // Least-squares slope in degC/min; needs >= 10 points spanning >= 90 s.
    bool slope(uint32_t now, double &out) const {
        (void)now;
        if (n_ < 10) return false;
        uint32_t t0 = tt_[head_];
        if (tt_[(head_ + n_ - 1) % N] - t0 < 90000) return false;
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int k = 0; k < n_; k++) {
            int i = (head_ + k) % N;
            double x = (tt_[i] - t0) / 60000.0, y = tv_[i];
            sx += x; sy += y; sxx += x * x; sxy += x * y;
        }
        double den = n_ * sxx - sx * sx;
        if (den <= 0) return false;
        out = (n_ * sxy - sx * sy) / den;
        return true;
    }
};

}  // namespace rules
