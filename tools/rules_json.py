#!/usr/bin/env python3
"""Print the module's thresholds from rules.h as JSON (the Pi dashboard's copy).

  python3 tools/rules_json.py > rules.json

pi/deploy.sh runs this, so the dashboard always shows the numbers the module
actually uses. Change thresholds in rules.h only, then reflash and redeploy.
"""
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# rules.h constant -> (dashboard key, scale)
MAP = {
    "GAS_REF_OHM": ("gas_ref_ohm", 1), "GAS_WARN_RATIO": ("gas_ratio_warn", 1),
    "GAS_CRIT_RATIO": ("gas_ratio_crit", 1), "GAS_DRIFT_RATIO": ("gas_drift_ratio", 1),
    "GAS_DRIFT_MS": ("gas_drift_min", 1 / 60000), "DP_WARN_PA": ("dp_warn_pa", 1), "DP_CRIT_PA": ("dp_crit_pa", 1),
    "T_WARN_C": ("t117_warn_c", 1), "T_CRIT_C": ("t117_crit_c", 1),
    "T_RISE_WARN_C_PER_MIN": ("t_rise_warn_c_per_min", 1), "COINCIDENCE_MS": ("coincidence_s", 1 / 1000),
}

src = open(os.path.join(ROOT, "rules.h")).read()
found = dict(re.findall(r"static const \w+ (\w+) = ([0-9.]+);", src))
missing = [k for k in MAP if k not in found]
if missing:
    sys.exit(f"rules.h: constants not found: {missing}")
print(json.dumps({key: round(float(found[c]) * scale, 6) for c, (key, scale) in MAP.items()}, indent=2))
