# Raspberry Pi gateway: BESS dashboard

The MAX32630 sends every `$BESS` frame (see `../bess_proto.h`) to the Pi over a
3.3 V UART as well as over USB. The Pi runs a small dashboard for live status and alerts.

**Pi:** a Raspberry Pi 5 (Raspberry Pi OS / Debian 13) reachable over SSH with your
key, with passwordless sudo. Set `BESS_PI_HOST` to its SSH host or `~/.ssh/config`
alias (or pass it to `pi/deploy.sh`). Ours browns out under heavy load, so the
service is kept light, and nothing here reboots the Pi.

## Wiring (115200 8N1, 3.3 V on both sides)

| MAX32630FTHR | Raspberry Pi 5 |
|---|---|
| P3_1 (UART2 TX) | pin 10, GPIO15 RXD |
| P3_0 (UART2 RX) | pin 8, GPIO14 TXD |
| GND | any GND, e.g. pin 6 |

The firmware switches P3_0/P3_1 to the 3.3 V I/O rail. If the wires are crossed
the other way, build with `PI_UART_SWAP 1` in `main.cpp`.

On the Pi, pins 8/10 are UART0, `/dev/ttyAMA0`. `/boot/firmware/config.txt` has
`dtparam=uart0=on` (original saved as `config.txt.bak-before-bess`). The login
console stays on the Pi 5's separate debug UART (`serial0` = `ttyAMA10`).
Without a reboot, `sudo dtparam uart0=on` enables it live.

## What runs on the Pi

| Item | Where |
|---|---|
| Service `bess-dashboard` | `/etc/systemd/system/bess-dashboard.service`, code in `/opt/bess-dashboard` |
| Web UI | nginx on port 80 and 443 (Laika certificate), proxied to `127.0.0.1:8080` |
| Logs | `/var/lib/bess-dashboard/bess_YYYY-MM-DD.csv` (every frame) and `events.jsonl` (alerts) |
| Thresholds | `/opt/bess-dashboard/config.json`, then `sudo systemctl restart bess-dashboard` |

The old `saturn-gateway` service was removed on 2026-10-15. Its unit file is kept
as `/opt/LIBRA-API-BE/saturn-gateway.service.removed-20261015`; the app in
`/opt/LIBRA-API-BE` was left in place. The old nginx site is
`/etc/nginx/sites-available/default.before-bess`.

## Deploy or update

```bash
ssh-add --apple-load-keychain          # if your SSH key has a passphrase in the keychain
export BESS_PI_HOST=<pi-ssh-host>      # e.g. pi@bess-gateway.local, or an alias from ~/.ssh/config
pi/deploy.sh
```

It copies the files, keeps an existing `config.json` (`pi/deploy.sh --replace-config`
overwrites it, saving the old one as `config.json.prev`), tests nginx (and restores
the previous site if the test fails), restarts the service and checks it answers.

## Alerts

Gas, enclosure pressure and air temperature are judged **on the module** (`../rules.h`,
see `../DETECTION.md`); the dashboard shows the module's results and doesn't re-judge
them. Their numbers come from `rules.h` at deploy time (`rules.json`), so the module
and the dashboard can't disagree. The Pi judges only humidity, the BSEC estimates
and the data link, with the limits in `config.json`.

| Alert | Level | Rule | Decided by |
|---|---|---|---|
| Possible battery event | critical | two of gas / pressure / temperature outside their limits within 1 minute, or air temperature ≥ 60 °C | module |
| Gas resistance low | warning / critical | 320 °C reading ≤ 60 / 35 % of the clean-air level (63.1 kΩ) | module |
| Gas resistance low for over an hour | warning | ≤ 80 % for 1 h: possible leak or cracked seal | module |
| Enclosure pressure out of range | warning / critical | \|ΔP\| ≥ 50 / 200 Pa | module |
| High air temperature | warning / critical | TMP117 ≥ 45 / 60 °C | module |
| Temperature rising fast | warning | ≥ 1 °C/min over 2 min | module |
| No data from the module | critical | no valid frame for 6 s | Pi |
| TMP117 / ABP2 / BME688 read failed | critical | firmware failure flag set | Pi (module flag) |
| Poor air quality | warning / critical | BSEC IAQ ≥ 150 / 250, once accuracy ≥ 1 | Pi |
| High CO2 equivalent | warning / critical | BSEC CO2-eq ≥ 1000 / 2000 ppm, once accuracy ≥ 1 (an estimate, not measured CO2) | Pi |
| High breath-VOC equivalent | warning / critical | BSEC bVOC-eq ≥ 2 / 5 ppm, once accuracy ≥ 1 | Pi |
| High humidity | warning | RH ≥ 85 % | Pi |
| BSEC not running | warning | firmware BSEC flag | Pi (module flag) |
| Corrupted frames | warning | ≥ 3 bad frames in a minute | Pi |

A single gas, pressure or temperature alert carries a hint about what it usually is
on its own (people or solvents; a door or HVAC; sun or heating).

Module reboots (and watchdog reboots) are logged as events.

## Dashboard tiles and charts

- **Temperature is the TMP117 only.** The BME688 chip and ABP2 die temperatures are
  still in the frames and the CSV logs, but not shown.
- **The battery event check** at the top shows the module's verdict and the three
  channels it combines.
- **Tile colour = the alert rules above**: OK (green), Warning (amber), Critical (red),
  each with an icon and label. Grey means not judged yet: "Calibrating" for the BSEC values
  at accuracy 0 (it restarts at each module boot).
  Each tile's gauge shows the value against its limits; barometric pressure has no limits.
- **Charts are averages**: 30 s buckets over 1 h, 2 min over 6 h (default), 10 min over 24 h,
  with a minimum vertical span so sensor noise doesn't fill the chart. Dashed lines are
  the alert limits when they fall in view. The last 24 h are reloaded from the CSVs after
  a service restart.

## API

`GET /api/status`, `GET /api/history?minutes=N&bucket=S` (up to 1440 min, S-second averages), `GET /api/events` (Server-Sent Events).
