#!/usr/bin/env python3
"""BESS dashboard service for the Raspberry Pi gateway.

Reads $BESS frames from the MAX32630 on the Pi's UART0 (GPIO14/15, pins 8/10),
keeps the live state and recent history, raises alerts, logs every frame to a
CSV per day, and serves the web dashboard plus a JSON/SSE API on 127.0.0.1
(nginx in front).

  GET /                      dashboard
  GET /api/status            latest frame, link state, active alerts, recent events
  GET /api/history?minutes=N&bucket=S
                             history averaged into S-second buckets (default 60 min, 30 s)
  GET /api/events            Server-Sent Events: "frame" and "alerts" messages

Only the standard library and pyserial (python3-serial). Frame parsing is the
same code as tools/bess_rx.py, deployed next to this file.
"""
import argparse
import csv
import glob
import json
import os
import queue
import statistics
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bess_rx import CSV_COLUMNS, FLAGS, LINK_TIMEOUT_S, FrameError, parse_frame  # noqa: E402

# Gas, pressure and temperature are judged ON THE MODULE (firmware rules.h, see
# DETECTION.md). Frames carry the result bits (field "rules") and flag 0x0020 for
# a possible battery event; the dashboard shows them and doesn't re-judge those
# channels. The numbers it displays come from rules.h via rules.json (pi/deploy.sh).
F_EVENT = 0x0020
R_GAS_WARN, R_GAS_CRIT, R_DP_WARN, R_DP_CRIT = 0x0001, 0x0002, 0x0004, 0x0008
R_T_WARN, R_T_CRIT, R_T_RISE, R_GAS_DRIFT, R_EVENT = 0x0010, 0x0020, 0x0040, 0x0080, 0x0100

HERE = os.path.dirname(os.path.abspath(__file__))

DEFAULTS = {
    "serial_port": "/dev/ttyAMA0",
    "baud": 115200,
    "http_host": "127.0.0.1",
    "http_port": 8080,
    "log_dir": os.environ.get("STATE_DIRECTORY", "/var/lib/bess-dashboard"),
    "history_minutes": 1440,
    "thresholds": {
        "iaq_warn": 150.0, "iaq_crit": 250.0,               # BSEC IAQ, only once accuracy >= 1
        "co2eq_warn_ppm": 1000.0, "co2eq_crit_ppm": 2000.0,  # BSEC estimate, accuracy >= 1
        "bvoc_warn_ppm": 2.0, "bvoc_crit_ppm": 5.0,          # BSEC estimate, accuracy >= 1
        "rh_warn_pct": 85.0,
        "bad_frames_per_min_warn": 3,
    },
}

LEVELS = {"info": 0, "warning": 1, "serious": 2, "critical": 3}

# What the dashboard charts. Air temperature is the TMP117 only; the BME688 chip
# and ABP2 die temperatures stay in the CSV logs but are not shown.
HIST_KEYS = ["seq", "t117_C", "dp_Pa", "gas_ohm", "iaq", "iaq_acc", "co2eq_ppm", "bvoc_ppm", "rh_pct", "p_Pa"]


def hist_row(f):
    """Compact history tuple. gas only at the 320 C steady heater step, else None."""
    row = [f.get(k) for k in HIST_KEYS]
    if f.get("heat_c") != 320 or (f.get("flags", 0) & 0x0008):
        row[HIST_KEYS.index("gas_ohm")] = None
    return tuple(row)
SENSOR_FLAGS = {0x0001: "TMP117", 0x0002: "ABP2", 0x0004: "BME688"}


def load_config(path):
    cfg = json.loads(json.dumps(DEFAULTS))
    if path and os.path.exists(path):
        with open(path) as f:
            user = json.load(f)
        th = user.pop("thresholds", {})
        cfg.update(user)
        cfg["thresholds"].update(th)
    # The module's thresholds (generated from rules.h at deploy). They are for
    # display; the module makes the decisions. Never overridden by config.json.
    rules_path = os.path.join(HERE, "rules.json")
    if os.path.exists(rules_path):
        with open(rules_path) as f:
            cfg["thresholds"].update(json.load(f))
    return cfg


class Hub:
    """Shared state: latest frame, history, alerts, events, SSE subscribers."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.th = cfg["thresholds"]
        self.lock = threading.Lock()
        self.started = time.time()
        self.latest = None
        self.last_ok = None
        self.link_ok = False
        self.port_open = False
        self.history = deque(maxlen=int(cfg["history_minutes"] * 60 / 2.5))  # (ts, hist_row)
        self.bad_times = deque(maxlen=200)
        self.stats = {"frames": 0, "bad": 0, "lost": 0, "reboots": 0}
        self.last_seq = None
        self.alerts = {}                         # id -> alert dict
        self.events = deque(maxlen=500)
        self.subs = []
        self.gas_ratio = None
        os.makedirs(cfg["log_dir"], exist_ok=True)
        self._csv_day = None
        self._csv_f = self._csv_w = None
        self._events_f = open(os.path.join(cfg["log_dir"], "events.jsonl"), "a")
        self._backfill()

    def _backfill(self):
        """Reload recent history from this service's own daily CSVs, so a
        restart doesn't empty the charts."""
        cutoff = time.time() - self.cfg["history_minutes"] * 60
        n = 0
        paths = [p for p in glob.glob(os.path.join(self.cfg["log_dir"], "bess_*.csv")) if os.path.getmtime(p) >= cutoff]
        for path in sorted(paths, key=os.path.getmtime):
            try:
                with open(path, newline="") as fh:
                    for r in csv.DictReader(fh):
                        try:
                            ts = datetime.fromisoformat(r["host_time"]).timestamp()
                        except (KeyError, ValueError):
                            continue
                        if ts < cutoff:
                            continue
                        f = {}
                        for k in HIST_KEYS + ["heat_c", "up_ms"]:
                            v = r.get(k, "")
                            f[k] = None if v in ("", None) else float(v)
                        f["flags"] = int(r.get("flags") or "0", 16)
                        row = hist_row(f)
                        self.history.append((ts, row))
                        n += 1
            except OSError:
                pass
        if n:
            print(f"backfilled {n} history rows from CSV", flush=True)

    # ---- SSE ----------------------------------------------------------------
    def subscribe(self):
        q = queue.Queue(maxsize=100)
        with self.lock:
            self.subs.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subs:
                self.subs.remove(q)

    def _broadcast(self, kind, payload):
        msg = f"event: {kind}\ndata: {json.dumps(payload)}\n\n"
        for q in list(self.subs):
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass  # slow client: it will resync from /api/status

    # ---- alerts ---------------------------------------------------------------
    def _event(self, level, msg, kind="event", alert_id=None):
        e = {"ts": time.time(), "level": level, "msg": msg, "kind": kind, "id": alert_id}
        self.events.append(e)
        self._events_f.write(json.dumps(e) + "\n")
        self._events_f.flush()
        return e

    def _raise(self, aid, level, title, detail=""):
        a = self.alerts.get(aid)
        if a and a["level"] == level:
            a["detail"] = detail
            return False
        verb = "raised" if not a else ("escalated" if LEVELS[level] > LEVELS[a["level"]] else "eased")
        self.alerts[aid] = {"id": aid, "level": level, "title": title, "detail": detail,
                            "since": a["since"] if a else time.time()}
        self._event(level, f"{title}: {detail}" if detail else title, verb, aid)
        return True

    def _clear(self, aid):
        a = self.alerts.pop(aid, None)
        if a:
            self._event("info", f"Cleared: {a['title']}", "cleared", aid)
            return True
        return False

    def _level_for(self, value, warn, crit, above=True):
        if value is None:
            return None
        if (value >= crit) if above else (value <= crit):
            return "critical"
        if (value >= warn) if above else (value <= warn):
            return "warning"
        return None

    def _set(self, aid, level, title, detail):
        return self._raise(aid, level, title, detail) if level else self._clear(aid)

    def _evaluate(self, f, now):
        th = self.th
        changed = False
        # sensor read failures (firmware flags)
        for bit, name in SENSOR_FLAGS.items():
            if f["flags"] & bit:
                changed |= self._raise(f"sensor_{name}", "critical", f"{name} read failed",
                                       "the module resets itself after 3 failures in a row")
            else:
                changed |= self._clear(f"sensor_{name}")
        if f["flags"] & 0x0080:
            changed |= self._raise("bsec", "warning", "BSEC not running", "IAQ / CO2-eq estimates unavailable")
        else:
            changed |= self._clear("bsec")
        # BSEC estimates, once BSEC has some confidence (accuracy >= 1)
        confident = (f.get("iaq_acc") or 0) >= 1
        iaq = f.get("iaq") if confident else None
        changed |= self._set("iaq", self._level_for(iaq, th["iaq_warn"], th["iaq_crit"]),
                             "Poor air quality (BSEC IAQ)", f"IAQ {iaq:.0f}" if iaq is not None else "")
        co2 = f.get("co2eq_ppm") if confident else None
        changed |= self._set("co2eq", self._level_for(co2, th["co2eq_warn_ppm"], th["co2eq_crit_ppm"]),
                             "High CO2 equivalent (BSEC estimate)", f"{co2:.0f} ppm" if co2 is not None else "")
        voc = f.get("bvoc_ppm") if confident else None
        changed |= self._set("bvoc", self._level_for(voc, th["bvoc_warn_ppm"], th["bvoc_crit_ppm"]),
                             "High breath-VOC equivalent (BSEC estimate)", f"{voc:.2f} ppm" if voc is not None else "")
        # humidity
        rh = f.get("rh_pct")
        changed |= self._set("rh", "warning" if rh is not None and rh >= th["rh_warn_pct"] else None,
                             "High humidity", f"{rh:.1f} %RH" if rh is not None else "")
        # gas / pressure / temperature: the module's own threshold results
        changed |= self._module_rules(f)
        # link quality
        while self.bad_times and now - self.bad_times[0] > 60:
            self.bad_times.popleft()
        n_bad = len(self.bad_times)
        changed |= self._set("bad_frames", "warning" if n_bad >= th["bad_frames_per_min_warn"] else None,
                             "Corrupted frames on the serial link", f"{n_bad} in the last minute")
        return changed

    def _module_rules(self, f):
        """Alerts from the module's rules bits. A single abnormal channel gets a
        hint from DETECTION.md about what it usually is."""
        r = f.get("rules")
        ids = ("t117", "t_rise", "dp", "gas", "gas_drift", "event")
        g = f.get("gas_ohm")
        if g and f.get("heat_c") == 320 and not (f["flags"] & 0x0008):
            self.gas_ratio = g / self.th["gas_ref_ohm"]
        if r is None:  # firmware before protocol v4: nothing judged on the module
            return any([self._clear(a) for a in ids])
        chans = [n for bit, n in ((R_GAS_WARN, "gas"), (R_DP_WARN, "pressure"), (R_T_WARN | R_T_RISE, "temperature"))
                 if r & bit]
        alone = len(chans) == 1 and not r & R_EVENT
        t, dp = f.get("t117_C"), f.get("dp_Pa")
        changed = False
        changed |= self._set("t117", "critical" if r & R_T_CRIT else "warning" if r & R_T_WARN else None,
                             "High air temperature", f"TMP117 {t:.2f} °C" if t is not None else "")
        changed |= self._set("t_rise", "warning" if r & R_T_RISE else None, "Temperature rising fast",
                             (f"TMP117 {t:.2f} °C" if t is not None else "") +
                             ("; temperature alone with no gas: often sun or heating" if alone else ""))
        changed |= self._set("dp", "critical" if r & R_DP_CRIT else "warning" if r & R_DP_WARN else None,
                             "Enclosure pressure out of range", (f"{dp:+.1f} Pa" if dp is not None else "") +
                             ("; pressure alone with no gas: often a door or HVAC" if alone else ""))
        gtxt = (f"{self.gas_ratio * self.th['gas_ref_ohm'] / 1000:.1f} kΩ = {self.gas_ratio * 100:.0f} % of the "
                f"clean-air level ({self.th['gas_ref_ohm'] / 1000:.1f} kΩ)") if self.gas_ratio else ""
        changed |= self._set("gas", "critical" if r & R_GAS_CRIT else "warning" if r & R_GAS_WARN else None,
                             "Gas resistance low (VOC / H2 / CO)",
                             gtxt + ("; gas alone with no pressure: often people, perfume or solvents" if alone else ""))
        changed |= self._set("gas_drift", "warning" if r & R_GAS_DRIFT else None,
                             "Gas resistance low for over an hour", "possible electrolyte leak or cracked seal: inspect")
        why = (" + ".join(chans) + " abnormal within a minute") if len(chans) >= 2 else "air temperature critical"
        changed |= self._set("event", "critical" if r & R_EVENT else None, "Possible battery event", why)
        return changed

    # ---- frames -------------------------------------------------------------------
    def on_frame(self, f, now):
        with self.lock:
            self.stats["frames"] += 1
            if self.last_seq is not None:
                if f["seq"] <= self.last_seq:
                    self.stats["reboots"] += 1
                    why = "after a watchdog reset" if f["flags"] & 0x0010 else ""
                    self._event("warning", f"Module rebooted {why}".strip(), "event")
                elif f["seq"] > self.last_seq + 1:
                    self.stats["lost"] += f["seq"] - self.last_seq - 1
            self.last_seq = f["seq"]
            self.last_ok = now
            if not self.link_ok:
                self.link_ok = True
                self._clear("link")
            f = dict(f, host_ts=now)
            self.latest = f
            self.history.append((now, hist_row(f)))
            self._evaluate(f, now)
            self._log_csv(f)
            payload = self._status_locked(now)
        self._broadcast("frame", payload)

    def on_bad(self, why, now):
        with self.lock:
            self.stats["bad"] += 1
            self.bad_times.append(now)

    def tick(self, now):
        """Called every second: link heartbeat."""
        with self.lock:
            ref = self.last_ok if self.last_ok is not None else self.started
            if now - ref > LINK_TIMEOUT_S:
                detail = ("no frame yet since the dashboard started" if self.last_ok is None
                          else f"last frame {now - self.last_ok:.0f} s ago")
                if self.link_ok or "link" not in self.alerts:
                    self.link_ok = False
                    self._raise("link", "critical", "No data from the module", detail)
                    payload = self._status_locked(now)
                else:
                    self.alerts["link"]["detail"] = detail
                    payload = None
            else:
                payload = None
        if payload:
            self._broadcast("frame", payload)

    def _log_csv(self, f):
        day = datetime.now().strftime("%Y-%m-%d")
        if day != self._csv_day:
            if self._csv_f:
                self._csv_f.close()
            path = os.path.join(self.cfg["log_dir"], f"bess_{day}.csv")
            n = 1
            while os.path.exists(path) and os.path.getsize(path) > 0:
                with open(path) as fh:  # a different column layout (older protocol): start a new file
                    if fh.readline().strip() == ",".join(CSV_COLUMNS):
                        break
                n += 1
                path = os.path.join(self.cfg["log_dir"], f"bess_{day}_{n}.csv")
            new = not os.path.exists(path) or os.path.getsize(path) == 0
            self._csv_f = open(path, "a", newline="")
            self._csv_w = csv.DictWriter(self._csv_f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
            if new:
                self._csv_w.writeheader()
            self._csv_day = day
        row = dict(f, host_time=datetime.now().isoformat(timespec="milliseconds"), flags=f"{f['flags']:04X}")
        if f.get("rules") is not None:
            row["rules"] = f"{f['rules']:04X}"
        self._csv_w.writerow(row)
        self._csv_f.flush()

    # ---- API views ------------------------------------------------------------------
    def _tile_states(self):
        """Colour for each dashboard tile: the worst active alert for that
        reading, 'pending' while it can't be judged yet, else 'good'."""
        f = self.latest or {}
        flags = f.get("flags", 0)
        worst = lambda ids: max((self.alerts[i]["level"] for i in ids if i in self.alerts),
                                key=lambda l: LEVELS[l], default=None)
        confident = (f.get("iaq_acc") or 0) >= 1
        spec = {
            "t117_C": (["t117", "t_rise", "sensor_TMP117"], f.get("t117_C") is not None and f.get("rules") is not None),
            "dp_Pa": (["dp", "sensor_ABP2"], f.get("dp_Pa") is not None and f.get("rules") is not None),
            "gas_ohm": (["gas", "gas_drift", "sensor_BME688"], self.gas_ratio is not None and f.get("rules") is not None),
            "iaq": (["iaq", "sensor_BME688", "bsec"], confident),
            "co2eq_ppm": (["co2eq", "sensor_BME688", "bsec"], confident),
            "bvoc_ppm": (["bvoc", "sensor_BME688", "bsec"], confident),
            "rh_pct": (["rh", "sensor_BME688"], f.get("rh_pct") is not None),
            "p_Pa": (["sensor_BME688"], f.get("p_Pa") is not None),
            "event": (["event"], f.get("rules") is not None),
        }
        out = {}
        for key, (ids, judgeable) in spec.items():
            lvl = worst(ids)
            if not self.link_ok:
                out[key] = "stale"
            elif lvl:
                out[key] = lvl
            else:
                out[key] = "good" if judgeable else "pending"
        return out

    def _status_locked(self, now):
        alerts = sorted(self.alerts.values(), key=lambda a: (-LEVELS[a["level"]], a["since"]))
        return {
            "now": now,
            "link": {"ok": self.link_ok, "port": self.cfg["serial_port"], "port_open": self.port_open,
                     "last_frame_age_s": None if self.last_ok is None else round(now - self.last_ok, 1)},
            "latest": self.latest,
            "gas_ref": self.th["gas_ref_ohm"],
            "gas_ratio": self.gas_ratio,
            "tiles": self._tile_states(),
            "rules": (self.latest or {}).get("rules"),            "alerts": alerts,
            "events": list(self.events)[-40:][::-1],
            "stats": dict(self.stats),
            "flags": {f"{k:04X}": v for k, v in FLAGS.items()},
            "thresholds": self.th,
        }

    def status(self):
        with self.lock:
            return self._status_locked(time.time())

    def history_view(self, minutes, bucket_s):
        """Mean of each reading per bucket_s-second bucket (empty buckets left
        out, so gaps stay visible). Smooths the per-frame sensor noise."""
        with self.lock:
            now = time.time()
            rows = [(ts, h) for ts, h in self.history if now - ts <= minutes * 60]
        buckets = {}
        for ts, h in rows:
            b = int(ts // bucket_s)
            acc = buckets.setdefault(b, [[0.0, 0] for _ in HIST_KEYS])
            for i, v in enumerate(h):
                if v is not None:
                    acc[i][0] += v
                    acc[i][1] += 1
        out = {"bucket_s": bucket_s, "t": []}
        for k in HIST_KEYS:
            out[k] = []
        for b in sorted(buckets):
            out["t"].append((b + 0.5) * bucket_s)
            for i, k in enumerate(HIST_KEYS):
                tot, n = buckets[b][i]
                out[k].append(None if not n else round(tot / n, 4))
        return out


def serial_reader(hub, stop):
    import serial  # pyserial
    port = hub.cfg["serial_port"]
    warned = False
    while not stop.is_set():
        try:
            ser = serial.Serial(port, hub.cfg["baud"], timeout=0.5)
        except (OSError, serial.SerialException) as e:
            if not warned:
                print(f"cannot open {port}: {e}; retrying", flush=True)
                warned = True
            hub.port_open = False
            time.sleep(2)
            continue
        print(f"reading {port} at {hub.cfg['baud']} baud", flush=True)
        warned = False
        hub.port_open = True
        pending = b""
        try:
            while not stop.is_set():
                pending += ser.readline()
                if not pending.endswith(b"\n"):
                    if len(pending) > 4096:
                        pending = b""
                    continue
                line, pending = pending.decode("ascii", "replace"), b""
                if "$BESS," not in line:
                    continue
                now = time.time()
                try:
                    hub.on_frame(parse_frame(line), now)
                except FrameError as e:
                    hub.on_bad(str(e), now)
        except (OSError, serial.SerialException) as e:
            print(f"serial error: {e}; reopening", flush=True)
            hub.port_open = False
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1)


def make_handler(hub):
    static_dir = os.path.join(HERE, "static")

    class Handler(BaseHTTPRequestHandler):
        server_version = "bess-dashboard"

        def log_message(self, fmt, *args):
            pass  # nginx logs requests

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/api/status":
                return self._json(hub.status())
            if u.path == "/api/history":
                q = parse_qs(u.query)
                try:
                    minutes = float(q.get("minutes", ["60"])[0])
                    bucket = float(q.get("bucket", ["30"])[0])
                except ValueError:
                    minutes, bucket = 60, 30
                minutes = max(1, min(minutes, hub.cfg["history_minutes"]))
                bucket = max(3, min(bucket, 3600))
                return self._json(hub.history_view(minutes, bucket))
            if u.path == "/api/events":
                return self._sse()
            if u.path in ("/", "/index.html"):
                return self._file(os.path.join(static_dir, "index.html"), "text/html; charset=utf-8")
            self._json({"error": "not found"}, 404)

        def _file(self, path, ctype):
            try:
                with open(path, "rb") as fh:
                    body = fh.read()
            except OSError:
                return self._json({"error": "missing"}, 404)
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _sse(self):
            q = hub.subscribe()
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            try:
                self.wfile.write(f"event: frame\ndata: {json.dumps(hub.status())}\n\n".encode())
                self.wfile.flush()
                while True:
                    try:
                        msg = q.get(timeout=15)
                    except queue.Empty:
                        msg = ": keepalive\n\n"
                    self.wfile.write(msg.encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                hub.unsubscribe(q)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(HERE, "config.json"))
    args = ap.parse_args()
    cfg = load_config(args.config)
    hub = Hub(cfg)
    stop = threading.Event()
    threading.Thread(target=serial_reader, args=(hub, stop), daemon=True).start()

    def ticker():
        while not stop.is_set():
            hub.tick(time.time())
            time.sleep(1)
    threading.Thread(target=ticker, daemon=True).start()

    srv = ThreadingHTTPServer((cfg["http_host"], cfg["http_port"]), make_handler(hub))
    srv.daemon_threads = True
    print(f"BESS dashboard on http://{cfg['http_host']}:{cfg['http_port']}, logging to {cfg['log_dir']}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()


if __name__ == "__main__":
    main()
