// On-board anomaly detection: the autoencoder trained by ml/train_autoencoder.py,
// running on the MAX32630. No mbed dependencies, so the same code is checked
// against the Python reference on a PC (ml/check_firmware_anomaly.py).
//
// Mirrors ml/bess_features.py exactly: same 8 features, same windows, same
// warm-up rules, fed with the same values the $BESS frame carries (0.01-unit
// fixed point), on the module's own clock (up_ms). The weights come from
// ml/anomaly_model.h, generated from ml/model.json.
//
// Score = mean squared reconstruction error / threshold (1.0 = the learned
// limit). The alarm (frame flag F_LOCAL_ALARM) sets after PERSIST frames above
// 1.0 and clears after CLEAR_FRAMES frames back below.
#pragma once
#include <math.h>
#include <stdint.h>
#include <algorithm>

#include "anomaly_model.h"   // namespace anomaly_model: weights, mean, std, threshold

namespace anomaly {

static const uint32_t LONG_MS = 1800000;   // 30-min median windows
static const uint32_t SLOPE_MS = 300000;   // 5-min slopes
static const uint32_t SHORT_MS = 60000;    // 1-min pressure window
static const uint32_t GAP_MS = 60000;      // a longer gap restarts the windows
static const int CLEAR_FRAMES = 10;        // 30 s back under the limit clears the alarm

struct Input {           // one frame's values, as sent in the frame
    uint32_t seq, up_ms;
    bool t_ok; double t117;       // degC
    bool dp_ok; double dp;        // Pa
    bool gas_ok; double gas;      // ohm
    uint16_t heat_c;
    bool gas_invalid;             // frame flag F_GAS_INVALID
    bool rh_ok; double rh;        // %RH
};

// Time-ordered (ms, value) ring buffer.
template <int N>
struct Ring {
    uint32_t t[N];
    double v[N];
    int head = 0, n = 0;
    void clear() { head = n = 0; }
    void push(uint32_t ms, double val) {
        int i = (head + n) % N;
        if (n == N) head = (head + 1) % N; else n++;   // full: drop the oldest
        t[i] = ms; v[i] = val;
    }
    uint32_t t_at(int k) const { return t[(head + k) % N]; }
    double v_at(int k) const { return v[(head + k) % N]; }
    // Same rule as the Python deque trim: drop while now - oldest > span.
    void trim(uint32_t now, uint32_t span) {
        while (n && now - t[head] > span) { head = (head + 1) % N; n--; }
    }
};

class Detector {
public:
    static const int CAP = 1024;   // 30 min at one frame per 3 s is ~600

    bool ready = false;            // a score is available this frame
    float ratio = 0;               // score / threshold
    bool alarm = false;            // persistent: drives F_LOCAL_ALARM
    float features[8];

    Detector() { reset(); }

    void reset() {
        t_.clear(); dp_.clear(); gas_.clear(); rh_.clear();
        have_first_ = have_last_ = false;
        ready = false;
    }

    // Returns true when a score was computed (ready).
    bool update(const Input &in) {
        uint32_t ms = in.up_ms;
        if (have_last_ && (ms - last_ms_ > GAP_MS || in.seq <= last_seq_)) reset();
        last_ms_ = ms; last_seq_ = in.seq; have_last_ = true;
        if (!have_first_) { first_ms_ = ms; have_first_ = true; }

        if (in.t_ok) t_.push(ms, in.t117);
        if (in.dp_ok) dp_.push(ms, in.dp);
        if (in.gas_ok && in.gas > 0 && in.heat_c == 320 && !in.gas_invalid) gas_.push(ms, log(in.gas));
        if (in.rh_ok) rh_.push(ms, in.rh);
        t_.trim(ms, LONG_MS); gas_.trim(ms, LONG_MS); rh_.trim(ms, LONG_MS); dp_.trim(ms, SHORT_MS);

        ready = compute(ms, in.up_ms);
        if (ready) {
            ratio = (float)(score() / anomaly_model::THRESHOLD);
            if (ratio > 1.0f) { above_++; below_ = 0; } else { below_++; above_ = 0; }
            if (above_ >= anomaly_model::PERSIST_FRAMES || (alarm && ratio > 1.0f)) alarm = true;
            else if (alarm && below_ >= CLEAR_FRAMES) alarm = false;
        } else {
            above_ = below_ = 0;
            alarm = false;
        }
        return ready;
    }

private:
    Ring<CAP> t_, gas_, rh_;
    Ring<64> dp_;
    uint32_t first_ms_ = 0, last_ms_ = 0, last_seq_ = 0;
    bool have_first_ = false, have_last_ = false;
    int above_ = 0, below_ = 0;
    double scratch_[CAP];

    template <int N>
    double median(const Ring<N> &r) {
        for (int k = 0; k < r.n; k++) scratch_[k] = r.v_at(k);
        std::sort(scratch_, scratch_ + r.n);
        return (r.n % 2) ? scratch_[r.n / 2] : (scratch_[r.n / 2 - 1] + scratch_[r.n / 2]) / 2.0;
    }

    // Least-squares slope per minute over the points within SLOPE_MS of now.
    template <int N>
    bool slope(const Ring<N> &r, uint32_t now, double &out) {
        int k0 = 0;
        while (k0 < r.n && now - r.t_at(k0) > SLOPE_MS) k0++;
        int n = r.n - k0;
        if (n < 10) return false;
        double t0 = r.t_at(k0), sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int k = k0; k < r.n; k++) {
            double x = (r.t_at(k) - t0) / 60000.0, y = r.v_at(k);
            sx += x; sy += y; sxx += x * x; sxy += x * y;
        }
        double den = n * sxx - sx * sx;
        if (den <= 0) return false;
        out = (n * sxy - sx * sy) / den;
        return true;
    }

    bool compute(uint32_t now, uint32_t up_ms) {
        if (now - first_ms_ < LONG_MS || up_ms < LONG_MS) return false;
        if (t_.n < 300 || gas_.n < 300 || rh_.n < 300 || dp_.n < 10) return false;
        double t_rate, gas_rate, rh_rate;
        if (!slope(t_, now, t_rate) || !slope(gas_, now, gas_rate) || !slope(rh_, now, rh_rate)) return false;
        double sum = 0, sq = 0;
        for (int k = 0; k < dp_.n; k++) sum += dp_.v_at(k);
        double mean = sum / dp_.n;
        for (int k = 0; k < dp_.n; k++) sq += (dp_.v_at(k) - mean) * (dp_.v_at(k) - mean);
        features[0] = (float)t_rate;
        features[1] = (float)(t_.v_at(t_.n - 1) - median(t_));
        features[2] = (float)mean;
        features[3] = (float)sqrt(sq / dp_.n);   // population std, like statistics.pstdev
        features[4] = (float)(gas_.v_at(gas_.n - 1) - median(gas_));
        features[5] = (float)gas_rate;
        features[6] = (float)rh_rate;
        features[7] = (float)(rh_.v_at(rh_.n - 1) - median(rh_));
        return true;
    }

    double score() {
        using namespace anomaly_model;
        float a[16], b[16];
        for (int i = 0; i < N_IN; i++) {
            float z = (features[i] - MEAN[i]) / STD[i];
            a[i] = z < -10 ? -10 : (z > 10 ? 10 : z);
        }
        float z0[16];
        for (int i = 0; i < N_IN; i++) z0[i] = a[i];
        const float *in = a;
        float *out = b;
        int w = 0, bi = 0;
        for (int L = 0; L < N_LAYERS; L++) {
            for (int o = 0; o < LAYER_OUT[L]; o++) {
                float s = BIAS[bi + o];
                for (int i = 0; i < LAYER_IN[L]; i++) s += WEIGHTS[w + o * LAYER_IN[L] + i] * in[i];
                out[o] = LAYER_TANH[L] ? tanhf(s) : s;
            }
            w += LAYER_OUT[L] * LAYER_IN[L];
            bi += LAYER_OUT[L];
            for (int o = 0; o < LAYER_OUT[L]; o++) a[o] = out[o];
            in = a;
        }
        double err = 0;
        for (int i = 0; i < N_IN; i++) err += (double)(a[i] - z0[i]) * (a[i] - z0[i]);
        return err / N_IN;
    }
};

}  // namespace anomaly
