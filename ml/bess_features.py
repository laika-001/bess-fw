"""Features and inference for the BESS anomaly autoencoder.

The same code runs in training (ml/train_autoencoder.py, over logged CSVs) and
live on the Pi (pi/bess_dashboard.py, frame by frame), so the features can't
drift apart. Pure Python: the Pi needs no extra packages to score.

The features describe behaviour, not absolute levels. Absolute limits (too hot,
pressure too high) are the dashboard's threshold alerts; the autoencoder looks
for combinations and rates of change it never saw during healthy operation:

  t_rate     TMP117 slope over 5 min                     degC/min
  t_dev      TMP117 minus its 30-min median              degC
  dp_mean    ABP2 gauge pressure, 1-min mean             Pa
  dp_std     ABP2 gauge pressure, 1-min std. deviation   Pa
  gas_ratio  ln(gas / its 30-min median), 320 C step     -
  gas_rate   slope of ln(gas) over 5 min                 1/min
  rh_rate    humidity slope over 5 min                   %RH/min
  rh_dev     humidity minus its 30-min median            %RH
"""
import json
import math
import statistics
from collections import deque

FEATURES = ["t_rate", "t_dev", "dp_mean", "dp_std", "gas_ratio", "gas_rate", "rh_rate", "rh_dev"]

# Smallest standard deviation used to scale each feature. A quiet night gives
# tiny spreads; without a floor, ordinary sensor noise would look like a big
# deviation. Roughly: "a change this size is not worth noticing on its own".
STD_FLOOR = {"t_rate": 0.05, "t_dev": 0.2, "dp_mean": 1.0, "dp_std": 0.3,
             "gas_ratio": 0.03, "gas_rate": 0.005, "rh_rate": 0.1, "rh_dev": 0.5}

LONG_S = 1800    # 30-min median windows
SLOPE_S = 300    # 5-min slopes
SHORT_S = 60     # 1-min pressure window
GAP_S = 60       # a longer gap in the data (link loss) starts the windows over


def _slope_per_min(pts):
    """Least-squares slope of [(t, v)] in units per minute."""
    n = len(pts)
    if n < 10:
        return None
    t0 = pts[0][0]
    sx = sy = sxx = sxy = 0.0
    for t, v in pts:
        x = (t - t0) / 60.0
        sx += x; sy += v; sxx += x * x; sxy += x * v
    den = n * sxx - sx * sx
    return None if den <= 0 else (n * sxy - sx * sy) / den


class FeatureTracker:
    """Feed frames in time order; get a feature vector once the windows are full.

    update() returns None during warm-up: the first 30 min after start, after a
    module reboot, or after a data gap, and while the module itself is in its
    gas warm-up (up_ms < 30 min).
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.t = deque()        # (ts, t117)
        self.dp = deque()       # (ts, dp)
        self.gas = deque()      # (ts, ln gas), 320 C readings only
        self.rh = deque()       # (ts, rh)
        self.first_ts = None
        self.last_ts = None
        self.last_seq = None

    @staticmethod
    def _trim(d, now, span):
        while d and now - d[0][0] > span:
            d.popleft()

    def update(self, ts, f):
        """f: dict with seq, up_ms, t117_C, dp_Pa, gas_ohm, heat_c, rh_pct, flags."""
        seq = f.get("seq")
        if (self.last_ts is not None and ts - self.last_ts > GAP_S) or \
           (self.last_seq is not None and seq is not None and seq <= self.last_seq):
            self.reset()
        self.last_ts, self.last_seq = ts, seq
        if self.first_ts is None:
            self.first_ts = ts

        if f.get("t117_C") is not None:
            self.t.append((ts, f["t117_C"]))
        if f.get("dp_Pa") is not None:
            self.dp.append((ts, f["dp_Pa"]))
        g = f.get("gas_ohm")
        if g and g > 0 and f.get("heat_c") == 320 and not (int(f.get("flags") or 0) & 0x0008):
            self.gas.append((ts, math.log(g)))
        if f.get("rh_pct") is not None:
            self.rh.append((ts, f["rh_pct"]))
        for d in (self.t, self.gas, self.rh):
            self._trim(d, ts, LONG_S)
        self._trim(self.dp, ts, SHORT_S)

        if ts - self.first_ts < LONG_S or (f.get("up_ms") or 0) < LONG_S * 1000:
            return None
        if len(self.t) < 300 or len(self.gas) < 300 or len(self.rh) < 300 or len(self.dp) < 10:
            return None

        recent = lambda d: [p for p in d if ts - p[0] <= SLOPE_S]
        t_rate = _slope_per_min(recent(self.t))
        gas_rate = _slope_per_min(recent(self.gas))
        rh_rate = _slope_per_min(recent(self.rh))
        if None in (t_rate, gas_rate, rh_rate):
            return None
        dpv = [v for _, v in self.dp]
        return [
            t_rate,
            self.t[-1][1] - statistics.median(v for _, v in self.t),
            sum(dpv) / len(dpv),
            statistics.pstdev(dpv),
            self.gas[-1][1] - statistics.median(v for _, v in self.gas),
            gas_rate,
            rh_rate,
            self.rh[-1][1] - statistics.median(v for _, v in self.rh),
        ]


class Autoencoder:
    """Inference only, from the model.json written by train_autoencoder.py."""

    def __init__(self, path):
        with open(path) as fh:
            m = json.load(fh)
        if m.get("features") != FEATURES:
            raise ValueError(f"model features {m.get('features')} != {FEATURES}")
        self.meta = m
        self.mean, self.std = m["mean"], m["std"]
        self.layers = m["layers"]           # [{"W": [[out x in]], "b": [out], "act": "tanh"|"linear"}]
        self.threshold = m["threshold"]

    def score(self, x):
        """Returns (score, per-feature squared error) for one raw feature vector.
        score = mean squared reconstruction error in standardised units."""
        z = [(v - m) / s for v, m, s in zip(x, self.mean, self.std)]
        z = [max(-10.0, min(10.0, v)) for v in z]   # one wild value shouldn't saturate everything
        h = z
        for L in self.layers:
            out = [sum(w * v for w, v in zip(row, h)) + b for row, b in zip(L["W"], L["b"])]
            h = [math.tanh(v) for v in out] if L["act"] == "tanh" else out
        err = [(a - b) ** 2 for a, b in zip(z, h)]
        return sum(err) / len(err), err
