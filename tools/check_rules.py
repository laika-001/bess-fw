#!/usr/bin/env python3
"""Replay logged frames through the firmware's threshold checks (rules.h).

  python3 tools/check_rules.py 'logs/*.csv'

Builds rules.h for this computer with clang++, feeds it every logged frame
(module clock, same values the frames carry) and prints each period where a
channel or the battery-event alarm was on. Use it to see what a threshold
change would have done before flashing it.
"""
import csv
import glob
import os
import subprocess
import sys
import tempfile
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAMES = [(0x0100, "BATTERY EVENT"), (0x0002, "gas critical"), (0x0001, "gas low"), (0x0008, "pressure critical"),
         (0x0004, "pressure"), (0x0020, "temp critical"), (0x0010, "temp high"), (0x0040, "temp rising"),
         (0x0080, "gas drift")]

HARNESS = r'''
#include <cstdio>
#include "rules.h"
int main() {
    rules::Checker c; rules::Input in;
    int t_ok, dp_ok, gas_ok, inval; unsigned seq, up, heat;
    while (scanf("%u %u %d %lf %d %lf %d %lf %u %d", &seq, &up, &t_ok, &in.t117, &dp_ok, &in.dp,
                 &gas_ok, &in.gas, &heat, &inval) == 10) {
        in.seq = seq; in.up_ms = up; in.t_ok = t_ok; in.dp_ok = dp_ok; in.gas_ok = gas_ok;
        in.heat_c = (uint16_t)heat; in.gas_invalid = inval;
        printf("%u %.3f %.3f\n", c.update(in), c.gas_ratio, c.t_rate_ok ? c.t_rate : 0.0);
    }
}
'''


def load(patterns):
    rows, seen = [], set()
    for pat in patterns:
        for path in sorted(glob.glob(pat)):
            with open(path, newline="") as fh:
                for r in csv.DictReader(fh):
                    if "heat_c" not in r or not r.get("up_ms"):
                        continue
                    key = (r["seq"], r["up_ms"])
                    if key in seen:
                        continue
                    seen.add(key)
                    rows.append(r)
    rows.sort(key=lambda r: r["host_time"])
    return rows


def main():
    pats = sys.argv[1:] or [os.path.join(ROOT, "logs", "*.csv")]
    rows = load(pats)
    if not rows:
        sys.exit("no frames with heat_c (protocol v2+) found")
    with tempfile.TemporaryDirectory() as tmp:
        src, exe = os.path.join(tmp, "h.cpp"), os.path.join(tmp, "h")
        open(src, "w").write(HARNESS)
        subprocess.run(["clang++", "-std=c++14", "-O2", "-Wall", "-I", ROOT, src, "-o", exe], check=True)
        num = lambda r, k: r.get(k) or ""
        lines = []
        for r in rows:
            f = lambda k: num(r, k)
            lines.append(" ".join(str(x) for x in [
                int(float(f("seq"))), int(float(f("up_ms"))),
                int(f("t117_C") != ""), f("t117_C") or 0, int(f("dp_Pa") != ""), f("dp_Pa") or 0,
                int(f("gas_ohm") != ""), f("gas_ohm") or 0, int(float(f("heat_c") or 0)),
                int(bool(int(r["flags"], 16) & 0x0008))]))
        out = subprocess.run([exe], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True).stdout.split()
    bits = [int(out[i]) for i in range(0, len(out), 3)]
    ratio = [float(out[i + 1]) for i in range(0, len(out), 3)]
    print(f"{len(rows)} frames, {rows[0]['host_time'][:16]} to {rows[-1]['host_time'][:16]}")
    for mask, name in NAMES:
        runs, start = [], None
        for r, b, g in zip(rows + [None], bits + [0], ratio + [0]):
            on = bool(b & mask)
            if on and start is None:
                start, worst = r, (r, g)
            if on and g >= 0 and g < worst[1]:
                worst = (r, g)
            if not on and start is not None:
                runs.append((start, prev, worst))
                start = None
            prev = r
        total = sum((datetime.fromisoformat(b["host_time"]) - datetime.fromisoformat(a["host_time"])).total_seconds() + 3
                    for a, b, _ in runs)
        print(f"\n{name}: {len(runs)} period(s), {total / 60:.1f} min in total")
        for a, b, (wr, wg) in runs[:12]:
            extra = []
            if mask & 0x0183:
                extra.append(f"gas min {float(wr['gas_ohm']) / 1000:.1f} kΩ ({wg * 100:.0f} %)" if wg >= 0 else "")
            if mask & 0x010C:
                extra.append(f"dP {a['dp_Pa']} Pa at start")
            if mask & 0x0170:
                extra.append(f"T {a['t117_C']} °C at start")
            print(f"  {a['host_time'][5:19]} - {b['host_time'][11:19]}  " + "  ".join(e for e in extra if e))
        if len(runs) > 12:
            print(f"  ... {len(runs) - 12} more")


if __name__ == "__main__":
    main()
