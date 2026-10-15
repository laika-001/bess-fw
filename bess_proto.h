// BESS data frame, protocol v4. No mbed or printf dependencies, so the same
// code is unit-tested on a PC.
//
// One ASCII line per sensor cycle (3 s, timed by BSEC), sent on the same serial
// port as the debug text. Receivers keep lines starting with "$BESS," and ignore the rest.
//
//   $BESS,<ver>,<seq>,<up_ms>,<flags>,<t117_cC>,<dp_cPa>,<tabp_cC>,<tbme_cC>,<p_Pa>,<rh_cP>,<gas_ohm>,
//         <heat_c>,<iaq_c>,<iaq_acc>,<siaq_c>,<co2eq_cppm>,<bvoc_ppb>,<bsec_st>,<rules>*<crc>\r\n
//
//   ver      protocol version (4). v1 frames (before BSEC) end after gas_ohm,
//            v2 after bsec_st, v3 had an autoencoder score where v4 has rules.
//   seq      frame counter, 0 at boot. A gap means lost frames; a drop back to 0 means a reboot
//   up_ms    milliseconds since the sensor loop started (~0.1 s after reset)
//   flags    4 hex digits, see Flag below
//   t117_cC  TMP117 temperature, 0.01 degC
//   dp_cPa   ABP2 gauge pressure, 0.01 Pa, zero-offset corrected (1 psi full scale = 689476)
//   tabp_cC  ABP2 die temperature, 0.01 degC
//   tbme_cC  BME688 temperature, 0.01 degC
//   p_Pa     BME688 absolute pressure, Pa
//   rh_cP    BME688 relative humidity, 0.01 %RH
//   gas_ohm  BME688 gas resistance, ohm
//   --- v2 ---
//   heat_c      BME688 heater target for this gas reading, degC. 320 in steady state;
//               BSEC's run-in uses 380/150/200 for the first 7 frames after boot
//   --- v2: Bosch BSEC2 estimates, all derived from the one gas resistance above ---
//   iaq_c       IAQ index, 0.01 (0..500; 50 = typical good air, 200 = polluted)
//   iaq_acc     BSEC accuracy 0..3 (0 = still stabilising, 3 = calibrated)
//   siaq_c      static IAQ (not scaled to the recent history), 0.01
//   co2eq_cppm  CO2-equivalent, 0.01 ppm. An ESTIMATE inferred from VOCs, not a CO2 measurement
//   bvoc_ppb    breath-VOC-equivalent, ppb (estimate)
//   bsec_st     bit 0 = stabilisation done, bit 1 = run-in done
//   The six BSEC fields are empty in cycles where BSEC gave no output.
//   --- v4: on-board threshold checks (rules.h, DETECTION.md) ---
//   rules       4 hex digits: bit 0 gas low, 1 gas critical, 2 pressure, 3 pressure
//               critical, 4 temperature high, 5 temperature critical, 6 temperature
//               rising fast, 7 gas drifting low for an hour, 8 possible battery event
//               (two channels within a minute, or temperature critical; also sets
//               F_LOCAL_ALARM).
//   crc      CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over every byte
//            between '$' and '*', exclusive, as 4 uppercase hex digits
//
// A field is empty (",,") when that reading is unavailable this cycle.
// Each frame is also the heartbeat: no valid frame for 6 s (longer than the
// 3 s cycle and the 5.6 s watchdog reset) means the module has failed or the link is down, and the
// receiver must treat that as a fault.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace bess {

static const int PROTO_VERSION = 4;
static const size_t FRAME_MAX = 224;

enum Flag : uint16_t {
    F_TMP_FAIL    = 0x0001,  // TMP117 read failed this cycle
    F_ABP_FAIL    = 0x0002,  // ABP2 read failed this cycle
    F_BME_FAIL    = 0x0004,  // BME688 measurement failed this cycle
    F_GAS_INVALID = 0x0008,  // gas measured, but valid / heater-stable bit not set
    F_WDT_BOOT    = 0x0010,  // this boot followed a watchdog reset
    F_LOCAL_ALARM = 0x0020,  // possible battery event (rules bit 8), decided on the module
    F_RELAY_OPEN  = 0x0040,  // dry-contact relay open (reserved)
    F_BSEC_ERR    = 0x0080,  // BSEC not running, or returned an error this cycle
};

struct Sample {
    uint32_t seq = 0;
    uint32_t up_ms = 0;
    uint16_t flags = 0;
    bool t117_ok = false; int32_t t117_cC = 0;
    bool abp_ok = false;  int32_t dp_cPa = 0; int32_t tabp_cC = 0;
    bool bme_ok = false;  int32_t tbme_cC = 0; int32_t p_Pa = 0; int32_t rh_cP = 0;
    bool gas_ok = false;  uint32_t gas_ohm = 0;   // gas_ok = a gas value is present
    uint16_t heat_c = 0;                          // heater target of the gas reading (with gas_ok)
    bool bsec_ok = false;                         // BSEC produced outputs this cycle
    int32_t iaq_c = 0; uint8_t iaq_acc = 0; int32_t siaq_c = 0;
    int32_t co2eq_cppm = 0; int32_t bvoc_ppb = 0; uint8_t bsec_st = 0;
    uint16_t rules = 0;                           // rules.h result bits
};

inline uint16_t crc16_ccitt(const char *p, size_t n)
{
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= (uint16_t)((uint8_t)*p++) << 8;
        for (int i = 0; i < 8; i++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

// Rounds a float to the nearest integer after scaling (e.g. degC -> 0.01 degC).
inline int32_t scaled(float v, float scale)
{
    float x = v * scale;
    return (int32_t)(x < 0 ? x - 0.5f : x + 0.5f);
}

class Writer {
public:
    Writer(char *buf, size_t cap) : b_(buf), cap_(cap) {}
    void ch(char c) { if (n_ + 1 < cap_) b_[n_++] = c; else over_ = true; }
    void str(const char *s) { while (*s) ch(*s++); }
    void u32(uint32_t v)
    {
        char t[10]; int k = 0;
        do { t[k++] = (char)('0' + v % 10); v /= 10; } while (v);
        while (k) ch(t[--k]);
    }
    void i32(int32_t v)
    {
        if (v < 0) { ch('-'); u32((uint32_t)(-(int64_t)v)); } else u32((uint32_t)v);
    }
    void hex16(uint16_t v)
    {
        static const char h[] = "0123456789ABCDEF";
        for (int s = 12; s >= 0; s -= 4) ch(h[(v >> s) & 0xF]);
    }
    size_t len() const { return n_; }
    bool overflow() const { return over_; }
    void terminate() { b_[n_ < cap_ ? n_ : cap_ - 1] = '\0'; }
private:
    char *b_; size_t cap_; size_t n_ = 0; bool over_ = false;
};

// Writes one complete frame (with CRC and CRLF) into buf.
// Returns its length, or 0 if buf is too small.
inline size_t encode(const Sample &s, char *buf, size_t cap)
{
    Writer w(buf, cap);
    w.str("$BESS,");
    w.u32(PROTO_VERSION); w.ch(',');
    w.u32(s.seq); w.ch(',');
    w.u32(s.up_ms); w.ch(',');
    w.hex16(s.flags); w.ch(',');
    if (s.t117_ok) w.i32(s.t117_cC);
    w.ch(',');
    if (s.abp_ok) w.i32(s.dp_cPa);
    w.ch(',');
    if (s.abp_ok) w.i32(s.tabp_cC);
    w.ch(',');
    if (s.bme_ok) w.i32(s.tbme_cC);
    w.ch(',');
    if (s.bme_ok) w.i32(s.p_Pa);
    w.ch(',');
    if (s.bme_ok) w.i32(s.rh_cP);
    w.ch(',');
    if (s.gas_ok) w.u32(s.gas_ohm);
    w.ch(',');
    if (s.gas_ok) w.u32(s.heat_c);
    w.ch(',');
    if (s.bsec_ok) w.i32(s.iaq_c);
    w.ch(',');
    if (s.bsec_ok) w.u32(s.iaq_acc);
    w.ch(',');
    if (s.bsec_ok) w.i32(s.siaq_c);
    w.ch(',');
    if (s.bsec_ok) w.i32(s.co2eq_cppm);
    w.ch(',');
    if (s.bsec_ok) w.i32(s.bvoc_ppb);
    w.ch(',');
    if (s.bsec_ok) w.u32(s.bsec_st);
    w.ch(',');
    w.hex16(s.rules);
    uint16_t crc = crc16_ccitt(buf + 1, w.len() - 1);
    w.ch('*');
    w.hex16(crc);
    w.str("\r\n");
    w.terminate();
    return w.overflow() ? 0 : w.len();
}

}  // namespace bess
