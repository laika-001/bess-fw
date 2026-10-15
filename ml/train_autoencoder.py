#!/usr/bin/env python3
"""Train the BESS anomaly autoencoder on healthy logged data.

  python3 ml/train_autoencoder.py --csv 'logs/*.csv' \\
      --train 2026-10-15T02:00 2026-10-15T07:30 \\
      --test  2026-10-15T07:30 2026-10-15T10:00

Frames from the CSVs (tools/bess_rx.py or the Pi dashboard logs) go through
ml/bess_features.py, the same code the Pi runs live. The network is
8 -> 6 -> 3 -> 6 -> 8 (tanh, linear output) trained to reproduce healthy
feature vectors; a high reconstruction error means "not like anything seen
while healthy". Only numpy is needed (Homebrew's python3 has it).

The alarm threshold is set from the healthy data: 1.2 x the 99.9th percentile
of the training and validation scores. The Pi raises a warning when the score
stays above it for 5 frames (15 s), critical above 3x.
"""
import argparse
import csv
import glob
import json
import os
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bess_features import FEATURES, STD_FLOOR, Autoencoder, FeatureTracker  # noqa: E402

HIDDEN = [6, 3, 6]
PERSIST = 5          # frames above threshold before the Pi alerts (matches the dashboard)
NUM = ["seq", "up_ms", "t117_C", "dp_Pa", "gas_ohm", "heat_c", "rh_pct"]


def load_frames(patterns):
    rows = []
    for pat in patterns:
        for path in sorted(glob.glob(pat)):
            with open(path, newline="") as fh:
                for r in csv.DictReader(fh):
                    if "heat_c" not in r:      # v1 frames: older firmware and heater; not used
                        continue
                    try:
                        ts = datetime.fromisoformat(r["host_time"]).timestamp()
                    except (KeyError, ValueError):
                        continue
                    f = {k: (float(r[k]) if r.get(k) not in ("", None) else None) for k in NUM}
                    f["flags"] = int(r.get("flags") or "0", 16)
                    rows.append((ts, f))
    rows.sort(key=lambda x: x[0])
    # the Mac logger and the Pi both log the same frames: keep one per (seq, up_ms)
    out, seen = [], set()
    for ts, f in rows:
        key = (f["seq"], f["up_ms"])
        if key not in seen:
            seen.add(key)
            out.append((ts, f))
    return out


def featurise(frames):
    """Feature vectors on the MODULE's clock (up_ms), as the firmware computes
    them; ts_out keeps the host time only to select the train/test windows."""
    tr = FeatureTracker()
    ts_out, X = [], []
    for ts, f in frames:
        x = tr.update(f["up_ms"] / 1000.0, f)
        if x is not None:
            ts_out.append(ts)
            X.append(x)
    return np.array(ts_out), np.array(X, dtype=np.float64)


class Net:
    def __init__(self, sizes, rng):
        self.W, self.b = [], []
        for a, b in zip(sizes[:-1], sizes[1:]):
            self.W.append(rng.normal(0, np.sqrt(2.0 / (a + b)), (b, a)))
            self.b.append(np.zeros(b))

    def forward(self, Z):
        hs = [Z]
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            a = hs[-1] @ W.T + b
            hs.append(a if i == len(self.W) - 1 else np.tanh(a))
        return hs

    def loss_grads(self, Z):
        hs = self.forward(Z)
        n = Z.shape[0]
        d = 2.0 * (hs[-1] - Z) / (n * Z.shape[1])
        loss = float(np.mean((hs[-1] - Z) ** 2))
        gW, gb = [None] * len(self.W), [None] * len(self.W)
        for i in reversed(range(len(self.W))):
            gW[i] = d.T @ hs[i]
            gb[i] = d.sum(axis=0)
            if i:
                d = (d @ self.W[i]) * (1 - hs[i] ** 2)
        return loss, gW, gb

    def scores(self, Z):
        return np.mean((self.forward(Z)[-1] - Z) ** 2, axis=1)


def train(Ztr, Zva, seed=0, epochs=600, batch=128, lr=2e-3, patience=40):
    rng = np.random.default_rng(seed)
    net = Net([Ztr.shape[1]] + HIDDEN + [Ztr.shape[1]], rng)
    params = net.W + net.b
    m = [np.zeros_like(p) for p in params]
    v = [np.zeros_like(p) for p in params]
    b1, b2, eps, step = 0.9, 0.999, 1e-8, 0
    best, best_params, bad = np.inf, None, 0
    for ep in range(epochs):
        idx = rng.permutation(len(Ztr))
        for s in range(0, len(idx), batch):
            _, gW, gb = net.loss_grads(Ztr[idx[s:s + batch]])
            step += 1
            for k, (p, g) in enumerate(zip(params, gW + gb)):
                m[k] = b1 * m[k] + (1 - b1) * g
                v[k] = b2 * v[k] + (1 - b2) * g * g
                p -= lr * (m[k] / (1 - b1 ** step)) / (np.sqrt(v[k] / (1 - b2 ** step)) + eps)
        val = float(np.mean(net.scores(Zva)))
        if val < best - 1e-6:
            best, best_params, bad = val, [p.copy() for p in params], 0
        else:
            bad += 1
            if bad >= patience:
                break
    for p, q in zip(params, best_params):
        p[...] = q
    return net, ep + 1, best


def events(ts, scores, thr, persist=PERSIST):
    """Runs of >= persist consecutive frames above thr -> [(start_ts, end_ts, peak, frames)]."""
    out, run = [], []
    for t, s in zip(ts, scores):
        if s > thr:
            run.append((t, s))
            continue
        if len(run) >= persist:
            out.append((run[0][0], run[-1][0], max(x[1] for x in run), len(run)))
        run = []
    if len(run) >= persist:
        out.append((run[0][0], run[-1][0], max(x[1] for x in run), len(run)))
    return out


def window(ts, X, start, end):
    a, b = datetime.fromisoformat(start).timestamp(), datetime.fromisoformat(end).timestamp()
    k = (ts >= a) & (ts < b)
    return ts[k], X[k]


def hhmm(t):
    return datetime.fromtimestamp(t).strftime("%H:%M:%S")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", nargs="+", required=True, help="CSV files or globs (quote globs)")
    ap.add_argument("--train", nargs=2, required=True, metavar=("FROM", "TO"), help="healthy period, ISO times")
    ap.add_argument("--test", nargs=2, metavar=("FROM", "TO"), help="period to replay and report on")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.json"))
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    frames = load_frames(args.csv)
    if not frames:
        sys.exit("no v2 frames found")
    ts, X = featurise(frames)
    print(f"{len(frames)} frames -> {len(X)} feature vectors (after 30-min warm-ups)")
    tts, Xt = window(ts, X, *args.train)
    if len(Xt) < 1000:
        sys.exit(f"only {len(Xt)} healthy feature vectors in the training window; need >= 1000 (~50 min)")
    hours = (tts[-1] - tts[0]) / 3600

    # standardise with floors, split by time: last 15 % of the healthy period validates
    mean = Xt.mean(axis=0)
    std = np.maximum(Xt.std(axis=0), [STD_FLOOR[f] for f in FEATURES])
    Z = np.clip((Xt - mean) / std, -10, 10)
    cut = int(len(Z) * 0.85)
    Ztr, Zva = Z[:cut], Z[cut:]

    t0 = time.time()
    net, epochs, val_loss = train(Ztr, Zva, seed=args.seed)
    s_tr, s_va = net.scores(Ztr), net.scores(Zva)
    thr = 1.2 * float(np.percentile(np.concatenate([s_tr, s_va]), 99.9))
    print(f"trained on {len(Ztr)} + {len(Zva)} validation vectors, {hours:.1f} h of healthy data, "
          f"{epochs} epochs in {time.time() - t0:.1f} s")
    print(f"mean score: train {s_tr.mean():.4f}, validation {s_va.mean():.4f}; threshold {thr:.4f}")
    ev_h = events(tts, np.concatenate([s_tr, s_va]), thr)
    print(f"healthy period: {(np.concatenate([s_tr, s_va]) > thr).mean() * 100:.2f} % of frames above threshold, "
          f"{len(ev_h)} alert(s) (>= {PERSIST} frames in a row)")

    model = {
        "version": 1,
        "created": datetime.now().isoformat(timespec="seconds"),
        "features": FEATURES,
        "mean": mean.tolist(),
        "std": std.tolist(),
        "layers": [{"W": W.tolist(), "b": b.tolist(), "act": "linear" if i == len(net.W) - 1 else "tanh"}
                   for i, (W, b) in enumerate(zip(net.W, net.b))],
        "threshold": thr,
        "persist_frames": PERSIST,
        "trained_on": {"from": args.train[0], "to": args.train[1], "hours": round(hours, 2),
                       "vectors": int(len(Z)), "files": sorted({os.path.basename(p) for pat in args.csv for p in glob.glob(pat)})},
        "healthy_alert_count": len(ev_h),
    }
    with open(args.out, "w") as fh:
        json.dump(model, fh, indent=1)
    print(f"wrote {args.out}")
    from export_model import export
    print(f"wrote {export(args.out, os.path.join(os.path.dirname(args.out), 'anomaly_model.h'))} "
          "(rebuild and flash the firmware to use it)")

    # the Pi's pure-Python scorer must agree with numpy
    ae = Autoencoder(args.out)
    py = [ae.score(x)[0] for x in Xt[:200]]
    print(f"pure-Python scorer vs numpy: max difference {max(abs(a - b) for a, b in zip(py, s_tr[:200])):.2e}")

    if args.test:
        sts, Xs = window(ts, X, *args.test)
        if not len(Xs):
            print("test window: no feature vectors")
            return
        Zs = np.clip((Xs - mean) / std, -10, 10)
        ss = net.scores(Zs)
        per = (net.forward(Zs)[-1] - Zs) ** 2
        ev = events(sts, ss, thr)
        print(f"\ntest {args.test[0]} -> {args.test[1]}: {len(Xs)} vectors, {(ss > thr).mean() * 100:.1f} % above "
              f"threshold, {len(ev)} alert(s):")
        for a, b, peak, n in ev:
            k = (sts >= a) & (sts <= b)
            top = np.argsort(-per[k].mean(axis=0))[:2]
            lvl = "CRITICAL" if peak > 3 * thr else "warning"
            print(f"  {hhmm(a)}-{hhmm(b)} ({n * 3} s) {lvl:8s} peak {peak / thr:5.1f}x threshold, "
                  f"mostly {FEATURES[top[0]]} + {FEATURES[top[1]]}")


if __name__ == "__main__":
    main()
