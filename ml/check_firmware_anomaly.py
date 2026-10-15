#!/usr/bin/env python3
"""Check the firmware's anomaly code (anomaly.h + ml/anomaly_model.h) against
the Python reference (ml/bess_features.py + ml/model.json), frame by frame.

  python3 ml/check_firmware_anomaly.py --csv 'logs/*.csv'

Builds anomaly.h for the Mac with clang++, feeds both implementations the same
logged frames on the module clock, and compares readiness, features, scores
and the alarm flag. Run it after every retrain, before flashing.

If the logs contain v3 frames (scored on the module), it also compares the
scores and alarm flags the MODULE sent with the Python reference.
"""
import csv
import glob
import argparse
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from bess_features import Autoencoder, FeatureTracker  # noqa: E402
from train_autoencoder import load_frames  # noqa: E402

HARNESS = r'''
#include <cstdio>
#include "anomaly.h"
int main() {
    anomaly::Detector d;
    anomaly::Input in;
    int t_ok, dp_ok, gas_ok, inval, rh_ok; unsigned seq, up, heat;
    while (scanf("%u %u %d %lf %d %lf %d %lf %u %d %d %lf", &seq, &up, &t_ok, &in.t117, &dp_ok, &in.dp,
                 &gas_ok, &in.gas, &heat, &inval, &rh_ok, &in.rh) == 12) {
        in.seq = seq; in.up_ms = up; in.t_ok = t_ok; in.dp_ok = dp_ok; in.gas_ok = gas_ok;
        in.heat_c = (uint16_t)heat; in.gas_invalid = inval; in.rh_ok = rh_ok;
        if (d.update(in)) {
            printf("1 %.9g %d", d.ratio, d.alarm);
            for (int i = 0; i < 8; i++) printf(" %.9g", d.features[i]);
            printf("\n");
        } else printf("0\n");
    }
}
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs="+", required=True)
    ap.add_argument("--model", default=os.path.join(HERE, "model.json"))
    args = ap.parse_args()

    frames = load_frames(args.csv)
    with tempfile.TemporaryDirectory() as tmp:
        src, exe = os.path.join(tmp, "h.cpp"), os.path.join(tmp, "h")
        open(src, "w").write(HARNESS)
        subprocess.run(["clang++", "-std=c++14", "-O2", "-Wall", "-I", ROOT, "-I", HERE, src, "-o", exe], check=True)
        lines = []
        for _, f in frames:
            v = lambda k: f.get(k)
            lines.append(" ".join(str(x) for x in [
                int(v("seq")), int(v("up_ms")),
                int(v("t117_C") is not None), v("t117_C") or 0, int(v("dp_Pa") is not None), v("dp_Pa") or 0,
                int(v("gas_ohm") is not None), v("gas_ohm") or 0, int(v("heat_c") or 0), int(bool(f["flags"] & 0x0008)),
                int(v("rh_pct") is not None), v("rh_pct") or 0]))
        out = subprocess.run([exe], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True).stdout.split("\n")

    ae = Autoencoder(args.model)
    persist = ae.meta.get("persist_frames", 5)
    tr = FeatureTracker()
    above = below = 0
    alarm = False
    n = ready_mismatch = alarm_mismatch = 0
    max_feat = max_rel = 0.0
    for (_, f), line in zip(frames, out):
        x = tr.update(f["up_ms"] / 1000.0, f)
        c = line.split()
        if (x is not None) != (c[0] == "1"):
            ready_mismatch += 1
            continue
        if x is None:
            above = below = 0
            alarm = False
            continue
        n += 1
        ratio = ae.score(x)[0] / ae.threshold
        if ratio > 1:
            above += 1; below = 0
        else:
            below += 1; above = 0
        if above >= persist or (alarm and ratio > 1):
            alarm = True
        elif alarm and below >= 10:
            alarm = False
        cf = [float(v) for v in c[3:]]
        max_feat = max(max_feat, max(abs(a - b) / max(abs(b), 1e-3) for a, b in zip(cf, x)))
        max_rel = max(max_rel, abs(float(c[1]) - ratio) / max(ratio, 1e-3))
        alarm_mismatch += (c[2] == "1") != alarm
    print(f"{len(frames)} frames, {n} scored by both")
    print(f"readiness mismatches: {ready_mismatch}")
    print(f"largest feature difference: {max_feat:.2e} (relative)")
    print(f"largest score difference:   {max_rel:.2e} (relative)")
    print(f"alarm flag mismatches:      {alarm_mismatch}")
    ok = ready_mismatch == 0 and alarm_mismatch == 0 and max_rel < 1e-3
    print("OK: firmware matches the Python reference" if ok else "MISMATCH")
    ok = check_device(args.csv, frames, ae) and ok
    sys.exit(0 if ok else 1)


def check_device(patterns, frames, ae):
    """Compare what the module itself sent (v3 anom_pct + alarm flag) with Python."""
    sent = {}
    for pat in patterns:
        for path in glob.glob(pat):
            with open(path, newline="") as fh:
                for r in csv.DictReader(fh):
                    if r.get("ver") == "3":
                        sent[(float(r["seq"]), float(r["up_ms"]))] = (
                            None if r.get("anom_pct", "") == "" else int(float(r["anom_pct"])),
                            bool(int(r["flags"], 16) & 0x0020))
    if not sent:
        print("\nno v3 frames in these logs: nothing scored on the module to compare yet")
        return True
    tr = FeatureTracker()
    persist = ae.meta.get("persist_frames", 5)
    above = below = 0
    alarm = False
    n = ready_mm = alarm_mm = 0
    worst = 0
    for _, f in frames:
        x = tr.update(f["up_ms"] / 1000.0, f)
        ratio = None if x is None else ae.score(x)[0] / ae.threshold
        if ratio is None:
            above = below = 0
            alarm = False
        else:
            if ratio > 1:
                above += 1; below = 0
            else:
                below += 1; above = 0
            if above >= persist or (alarm and ratio > 1):
                alarm = True
            elif alarm and below >= 10:
                alarm = False
        key = (f["seq"], f["up_ms"])
        if key not in sent:
            continue
        pct, flag = sent[key]
        if (pct is None) != (ratio is None):
            ready_mm += 1
            continue
        if pct is None:
            continue
        n += 1
        worst = max(worst, abs(pct - round(ratio * 100)))
        alarm_mm += flag != alarm
    print(f"\nmodule-scored frames compared: {n}")
    print(f"  warm-up mismatches: {ready_mm}, largest anom_pct difference: {worst} %-points, "
          f"alarm flag mismatches: {alarm_mm}")
    ok = ready_mm == 0 and alarm_mm == 0 and worst <= 1
    print("OK: the module's scores match the Python reference" if ok else "MISMATCH on the module")
    return ok


if __name__ == "__main__":
    main()
