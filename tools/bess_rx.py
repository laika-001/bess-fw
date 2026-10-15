#!/usr/bin/env python3
"""Gateway receiver for BESS protocol v1/v2 frames (see bess_proto.h).

Reads the module's serial port, keeps lines with valid "$BESS," frames, checks
the CRC, tracks the sequence number and the 6 s heartbeat timeout, and appends
each reading to a CSV in physical units.

  python3 bess_rx.py                          # auto-detects the port
  python3 bess_rx.py --port /dev/ttyACM0 --csv healthy.csv
  python3 bess_rx.py --stdin < saved.log      # replay a captured log (no timeout)
  python3 bess_rx.py --log-dir logs           # one CSV + raw text file per module boot

With --log-dir the receiver keeps running across reflashes and replugs: a new
pair of files starts at launch and whenever the module reboots (seq drops back),
so every file covers one boot and starts with that boot's sensor warm-up.
  logs/bess_<date>_<time>.csv      decoded frames, as with --csv
  logs/bess_<date>_<time>.raw.txt  every line received (debug text too), timestamped
If the port disappears it reconnects; opening it resets the module (DAPLink).

Needs pyserial for live ports: `pip install pyserial` (or apt install python3-serial on a Pi).
"""
import argparse
import csv
import glob
import os
import sys
import time
from datetime import datetime

PROTO_VERSIONS = (1, 2, 3, 4) # v2 adds BSEC2, v3 had an autoencoder score, v4 the on-board threshold results
BOOT_BANNER = "=== BESS:"     # first line the firmware prints after every reset
LINK_TIMEOUT_S = 6.0          # > 5.6 s watchdog reset period on the module

FLAGS = {
    0x0001: "TMP_FAIL",
    0x0002: "ABP_FAIL",
    0x0004: "BME_FAIL",
    0x0008: "GAS_INVALID",
    0x0010: "WDT_BOOT",
    0x0020: "BATTERY_EVENT",
    0x0040: "RELAY_OPEN",
    0x0080: "BSEC_ERR",
}

# (name, scale to physical units, unit) for the data fields, in frame order.
# v1 frames carry the first 7; v2 adds the BSEC2 estimates.
FIELDS_V1 = [
    ("t117_C", 0.01, "degC"),
    ("dp_Pa", 0.01, "Pa"),
    ("tabp_C", 0.01, "degC"),
    ("tbme_C", 0.01, "degC"),
    ("p_Pa", 1.0, "Pa"),
    ("rh_pct", 0.01, "%RH"),
    ("gas_ohm", 1.0, "ohm"),
]
FIELDS_V2 = FIELDS_V1 + [
    ("heat_c", 1, "degC"),              # heater target of gas_ohm (320 steady; run-in differs)
    ("iaq", 0.01, ""),                  # BSEC IAQ index 0..500
    ("iaq_acc", 1, ""),                 # 0..3
    ("siaq", 0.01, ""),                 # static IAQ
    ("co2eq_ppm", 0.01, "ppm"),         # ESTIMATE from VOCs, not measured CO2
    ("bvoc_ppm", 0.001, "ppm"),         # breath-VOC-equivalent estimate
    ("bsec_st", 1, ""),                 # bit0 stabilised, bit1 run-in done
]
FIELDS_V3 = FIELDS_V2 + [
    ("anom_pct", 1, "%"),               # on-board autoencoder score, % of the learned limit
]
FIELDS_V4 = FIELDS_V2 + [
    ("rules", "hex", ""),               # rules.h result bits (see RULE_BITS)
]
FIELDS_BY_VERSION = {1: FIELDS_V1, 2: FIELDS_V2, 3: FIELDS_V3, 4: FIELDS_V4}
FIELDS_LATEST = FIELDS_V4
RULE_BITS = {0x0001: "GAS_LOW", 0x0002: "GAS_CRIT", 0x0004: "DP", 0x0008: "DP_CRIT", 0x0010: "T_HIGH",
             0x0020: "T_CRIT", 0x0040: "T_RISE", 0x0080: "GAS_DRIFT", 0x0100: "BATTERY_EVENT"}
CSV_COLUMNS = ["host_time", "ver", "seq", "up_ms", "flags"] + [f[0] for f in FIELDS_LATEST]


def crc16_ccitt(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


class FrameError(ValueError):
    pass


def parse_frame(line: str) -> dict:
    """Parses one frame line. Returns a dict, or raises FrameError."""
    start = line.find("$BESS,")
    if start < 0:
        raise FrameError("no frame")
    line = line[start:].strip()
    star = line.rfind("*")
    if star < 0 or len(line) != star + 5:
        raise FrameError("no checksum")
    body = line[1:star]
    try:
        want = int(line[star + 1:], 16)
    except ValueError:
        raise FrameError("bad checksum digits")
    got = crc16_ccitt(body.encode("ascii", "replace"))
    if got != want:
        raise FrameError(f"CRC mismatch (got {got:04X}, frame says {want:04X})")
    parts = body.split(",")
    try:
        ver = int(parts[1])
    except (IndexError, ValueError):
        raise FrameError("bad version field")
    if ver not in FIELDS_BY_VERSION:
        raise FrameError(f"unsupported protocol version {parts[1]}")
    fields = FIELDS_BY_VERSION[ver]
    if len(parts) != 5 + len(fields):
        raise FrameError(f"v{ver}: expected {5 + len(fields)} fields, got {len(parts)}")
    out = {
        "ver": ver,
        "seq": int(parts[2]),
        "up_ms": int(parts[3]),
        "flags": int(parts[4], 16),
    }
    for (name, scale, _), raw in zip(fields, parts[5:]):
        if raw == "":
            out[name] = None
        elif scale == "hex":
            out[name] = int(raw, 16)
        elif isinstance(scale, int):
            out[name] = int(raw) * scale
        else:
            out[name] = round(int(raw) * scale, 3)
    for name, _, _ in FIELDS_LATEST:
        out.setdefault(name, None)  # fields this frame version doesn't carry
    return out


def flag_names(flags: int) -> str:
    return "|".join(n for bit, n in FLAGS.items() if flags & bit) or "-"


def find_port():
    ports = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/cu.usbmodem*"))
    return ports[0] if ports else None


class Link:
    """Sequence and heartbeat tracking."""

    def __init__(self, start=None):
        self.last_seq = None
        self.last_ok = start  # live ports: the 6 s clock starts when the port opens
        self.frames = self.bad = self.dropped = self.reboots = 0
        self.lost = False

    def on_frame(self, f, now):
        """Returns True if this frame shows the module rebooted."""
        rebooted = False
        if self.last_seq is not None:
            if f["seq"] <= self.last_seq:
                rebooted = True
                self.reboots += 1
                print(f"!! module rebooted (seq {self.last_seq} -> {f['seq']}, flags {flag_names(f['flags'])})")
            elif f["seq"] > self.last_seq + 1:
                gap = f["seq"] - self.last_seq - 1
                self.dropped += gap
                print(f"!! {gap} frame(s) lost before seq {f['seq']}")
        if self.lost:
            print("== LINK RESTORED")
            self.lost = False
        self.last_seq = f["seq"]
        self.last_ok = now
        self.frames += 1
        return rebooted

    def check_timeout(self, now):
        if self.last_ok is not None and not self.lost and now - self.last_ok > LINK_TIMEOUT_S:
            self.lost = True
            what = "since port opened" if self.last_seq is None else "since last frame"
            print(f"!! LINK LOST: no valid frame for {LINK_TIMEOUT_S:.0f} s ({what}) -> FAULT (fail-safe)")
            return True
        return False


class Sinks:
    """CSV (and raw text) output: one fixed --csv file, or rotating files in --log-dir."""

    def __init__(self, csv_path=None, log_dir=None):
        self.csv_path, self.log_dir = csv_path, log_dir
        self.fcsv = self.writer = self.fraw = None
        self.rows = 0
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self.open_new("start")

    def open_new(self, why):
        self.close()
        self.rows = 0
        if self.log_dir:
            stem = os.path.join(self.log_dir, "bess_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
            path = stem + ".csv"
            n = 1
            while os.path.exists(path):  # two boots within one second
                path = f"{stem}_{n}.csv"; n += 1
            self.fraw = open(path[:-4] + ".raw.txt", "a")
            print(f"== {why}: logging to {path} (+ .raw.txt)")
        elif self.csv_path:
            path = self.csv_path
        else:
            return
        new = not os.path.exists(path) or os.path.getsize(path) == 0
        self.fcsv = open(path, "a", newline="")
        # extrasaction="ignore": fields of older protocol versions (e.g. v3 anom_pct) aren't columns now
        self.writer = csv.DictWriter(self.fcsv, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        if new:
            self.writer.writeheader()

    def raw(self, line):
        if self.fraw:
            self.fraw.write(datetime.now().isoformat(timespec="milliseconds") + " " + line.rstrip("\r\n") + "\n")
            self.fraw.flush()

    def row(self, f):
        if self.writer:
            r = dict(f, host_time=datetime.now().isoformat(timespec="milliseconds"))
            r["flags"] = f"{f['flags']:04X}"
            if r.get("rules") is not None:
                r["rules"] = f"{r['rules']:04X}"
            self.writer.writerow(r)
            self.fcsv.flush()
            self.rows += 1

    def close(self):
        for fh in (self.fcsv, self.fraw):
            if fh:
                fh.close()
        self.fcsv = self.writer = self.fraw = None


def fmt(v, nd=2):
    return "  --  " if v is None else f"{v:.{nd}f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first /dev/ttyACM* or /dev/cu.usbmodem*)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", help="append readings to this CSV file")
    ap.add_argument("--log-dir", help="write one CSV + raw text file per module boot into this folder")
    ap.add_argument("--stdin", action="store_true", help="read lines from stdin instead of a port")
    ap.add_argument("--debug", action="store_true", help="also print the module's debug text")
    ap.add_argument("--quiet", action="store_true", help="don't print each reading")
    args = ap.parse_args()

    if args.csv and args.log_dir:
        sys.exit("Use --csv or --log-dir, not both.")
    live = not args.stdin
    if live:
        import serial  # pyserial

    def open_port():
        port = args.port or find_port()
        if not port:
            return None
        try:
            s = serial.Serial(port, args.baud, timeout=0.5)
        except (OSError, serial.SerialException):
            return None
        if not args.log_dir:
            # Single-file mode: drop frames queued before we opened (e.g. from
            # before a reflash). Not with --log-dir: opening the port resets the
            # module, its boot messages arrive within milliseconds and would be
            # lost, and stale lines land in the previous boot's file anyway.
            s.reset_input_buffer()
        print(f"== port open: {port}")
        return s

    ser = None
    if live:
        ser = open_port()
        if ser is None and not args.log_dir:
            sys.exit("No serial port found. Plug in the board or pass --port.")
    src = "stdin" if not live else (ser.port if ser else "(waiting for port)")

    print(f"BESS rx: protocol v1-v4, source {src}" + (f", logging to {args.csv}" if args.csv else ""))
    sinks = Sinks(args.csv, args.log_dir)
    link = Link(start=time.time() if live else None)
    pending = b""
    booted = False
    port_warned = False
    try:
        while True:
            if not live:
                raw = sys.stdin.readline()
                if not raw:
                    break
                line = raw
            else:
                if ser is None:  # port gone: keep the heartbeat running and retry
                    link.check_timeout(time.time())
                    time.sleep(1.0)
                    ser = open_port()
                    pending = b""
                    continue
                try:
                    pending += ser.readline()
                except (OSError, serial.SerialException) as e:
                    if not port_warned:
                        print(f"!! port lost ({e.__class__.__name__}); reconnecting")
                        port_warned = True
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                    continue
                if not pending.endswith(b"\n"):
                    line = ""  # nothing, or a partial line: wait for the rest
                    if len(pending) > 4096:
                        pending = b""  # no newline for 4 KB: noise, drop it
                else:
                    line, pending = pending.decode("ascii", "replace"), b""
            now = time.time()
            if live:
                link.check_timeout(now)
            if not line:
                continue
            if args.log_dir and BOOT_BANNER in line and sinks.rows:
                sinks.open_new("module booted")
                booted = True  # this boot already has its file; the seq drop below won't rotate again
            if "$BESS," not in line:
                sinks.raw(line)  # frame lines are written below, after any rotation
            if "$BESS," not in line:
                if args.debug and line.strip():
                    print("  dbg| " + line.rstrip())
                continue
            try:
                f = parse_frame(line)
            except FrameError as e:
                sinks.raw(line)
                link.bad += 1
                print(f"!! bad frame ({e}): {line.strip()[:100]}")
                continue
            if link.on_frame(f, now) and args.log_dir and not booted and sinks.rows:
                sinks.open_new("module rebooted")  # reboot without a banner seen (e.g. banner line lost)
            sinks.raw(line)
            booted = False
            port_warned = False
            sinks.row(f)
            if not args.quiet:
                print(f"#{f['seq']:<6} T117 {fmt(f['t117_C'])} C | dP {fmt(f['dp_Pa'])} Pa | "
                      f"Tbme {fmt(f['tbme_C'])} C  P {fmt(f['p_Pa'], 0)} Pa  RH {fmt(f['rh_pct'])} % | "
                      f"gas {fmt(f['gas_ohm'], 0)} ohm"
                      + (f" | IAQ {fmt(f['iaq'], 1)} acc {f['iaq_acc']}  CO2eq~{fmt(f['co2eq_ppm'], 0)} ppm"
                         f"  bVOC~{fmt(f['bvoc_ppm'], 2)} ppm" if f.get("iaq") is not None else "")
                      + (" | rules " + "|".join(n for b, n in RULE_BITS.items() if f["rules"] & b)
                         if f.get("rules") else "")
                      + f" | {flag_names(f['flags'])}")
    except KeyboardInterrupt:
        pass
    finally:
        sinks.close()
        print(f"frames ok={link.frames} bad={link.bad} lost={link.dropped} reboots={link.reboots}")


if __name__ == "__main__":
    main()
