# Detection: thermal-runaway signatures and the threshold rules

The module judges three channels every 3 s against fixed thresholds (`rules.h`)
and raises the battery-event alarm on the module itself. This file is the basis
for those rules.

## Thermal runaway types and their sensor signatures

| Trigger | What happens in the cell | Gas / VOC (BME688) | Enclosure ΔP (ABP2) | Air temp (TMP117) | Order and warning time | What the module sees |
|---|---|---|---|---|---|---|
| Thermal abuse / cooling failure | Whole pack warms; the protective layer on the electrodes breaks down; electrolyte evaporates, then the cell vents | ↓ then ↓↓ at vent | ↑ at vent | ↑ slow, steady from the start | T → gas → ΔP. Minutes to hours | Temperature-rate rule first; alarm when gas joins in |
| Overcharge | Electrolyte breaks down into gas, the cell swells, heats from within, then vents | ↓↓ at vent | ↑↑ pulse | ↑ moderate to fast | Gas and ΔP before a big air-temp rise. Minutes | Gas + ΔP + rising T: strongest combined signature |
| Internal short (defect, dendrite, separator failure) | Local hot spot inside one cell; little outside sign until it vents | ↓↓ sudden | ↑↑ sudden spike | ↑↑ steep, after the gas | Gas ≈ ΔP → T. Seconds to minutes | Off-gas is the earliest outside sign |
| External short | Very high current heats the whole cell | ↓ after heating | ↑ at vent | ↑↑ fast | T → gas → ΔP. Seconds to minutes | Mainly the BMS's job; module is an independent backup |
| Mechanical abuse (crush, puncture, drop) | Immediate internal short | ↓↓ at once | ↑↑ at once | ↑↑ quickly | Near-simultaneous. Seconds | All three channels at once: highest confidence |
| Over-discharge, then recharge | Copper dissolves and re-plates, a hidden fault that later shorts | – until failure | – until failure | – until failure | Silent for days or weeks, then like an internal short | Caught at the venting stage |
| Electrolyte leak / cracked seal | Slow solvent vapour, no heating yet | ↓ slow, persistent | – | – | Hours to days ahead | Gas-only drift: early maintenance warning |

## Telling a battery event from everyday disturbances

| Disturbance | Gas | ΔP | Temp | RH | How to tell |
|---|---|---|---|---|---|
| People, perfume, cleaning solvents | ↓ (can be large) | – | – | ↑ slight | Gas only, no pressure; recovers in minutes |
| Door opening, HVAC switching | – / slight | step, either way | step | step | ΔP step without gas |
| Sun or daytime heating | – | – | ↑ slow, follows time of day | ↓ | Daily cycle, no gas |
| Condensation / water ingress | slight drift | – | – | ↑↑ | Humidity-led; a separate hazard |

## The rules (`rules.h`, decided on the module)

| Rule | Threshold | Level |
|---|---|---|
| Gas low | gas resistance at the 320 °C heater step ≤ 60 % of its clean-air level (63.1 kΩ) | warning; ≤ 35 % critical |
| Gas drift | ≤ 80 % of the clean-air level continuously for 1 hour | maintenance warning (possible leak or cracked seal) |
| Enclosure pressure | \|ΔP\| ≥ 50 Pa | warning; ≥ 200 Pa critical |
| Air temperature | TMP117 ≥ 45 °C | warning; ≥ 60 °C critical |
| Temperature rising | TMP117 rises ≥ 1 °C/min over the last 2 min | warning |
| **Possible battery event** | **two channels (gas, pressure, temperature) outside their limits within 1 minute, or air temperature ≥ 60 °C** | **alarm: frame flag F_LOCAL_ALARM** |

One channel alone is a warning, not the alarm, because on its own it is
usually one of the everyday disturbances above. Every runaway type in the first
table disturbs at least two channels by the time the cell vents.

The clean-air level is the median gas resistance of the healthy night of
2026-10-15 (02:00-07:30, range 54.5-66.7 kΩ). Re-measure it in clean air after
replacing or re-mounting the BME688. Humidity (condensation) and the BSEC
estimates are judged by the Pi dashboard, not the module.

To change a threshold: edit `rules.h`, replay the logs with
`python3 tools/check_rules.py 'logs/*.csv'` to see what it would have done,
then `bessflash` and `pi/deploy.sh` (the dashboard reads the numbers from `rules.h`).

## Replay of the first day of logs

Over 10.5 hours of real data (2026-10-15, 01:11-11:15) the rules raised **no
battery-event alarm**. Every disturbance stayed on one channel:

- gas: two dips (to 5.7 kΩ and 15.1 kΩ, 9 % and 24 %) around 10:13 and 10:28;
- pressure: spikes of 201, 4196 and 274 Pa between 09:55 and 09:59;
- temperature and gas drift: none.

## Why not the autoencoder (yet)

An on-board autoencoder was prototyped (in git history up to commit 349c6ce).
Trained on one quiet night, it went into constant alarm by late morning when
the air was simply cleaner than during training. **After real testing on battery
failure signatures we will decide whether an autoencoder is the better
approach.** Until then the module uses these explicit thresholds, which can be
read, checked against the tables above, and tuned.
