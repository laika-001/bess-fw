# BESS enclosure monitor

Firmware and gateway for a battery energy storage (BESS) enclosure monitor.

- **Module** (MAX32630FTHR, mbed-os): reads a TMP117 (air temperature), a Honeywell
  ABP2 (enclosure gauge pressure) and a Bosch BME688 (gas, humidity, barometric
  pressure; IAQ / CO₂-equivalent / breath-VOC estimates through Bosch BSEC2), runs
  an anomaly-detection autoencoder on board, and sends one checked data frame every
  3 s over USB and UART. A hardware watchdog resets it if a sensor or the firmware stops.
- **Gateway** (Raspberry Pi 5): live web dashboard with alerts, trends and CSV logs.
- **Model training**: trains the autoencoder on healthy logged data and
  generates the weights the firmware compiles in.

| Where | What |
|---|---|
| [SETUP.md](SETUP.md) | Setup, hardware wiring, daily commands, board gotchas, data protocol |
| [pi/README.md](pi/README.md) | Raspberry Pi dashboard: wiring, service, deploy, alert rules |
| [ml/README.md](ml/README.md) | Anomaly autoencoder: features, training, checking, flashing |
| [third_party/README.md](third_party/README.md) | Third-party code and licenses |

macOS was used for this project.

## Quick start

```bash
git clone https://github.com/laika-001/bess-fw.git && cd bess-fw
tools/setup_mac.sh      # toolchain, Python tools, mbed-os, Bosch BSEC2 (asks to accept Bosch's license)
bessflash               # in a new terminal, with the board plugged in
bessmon                 # serial console
```

## Bosch BSEC2 is not redistributed

The firmware uses Bosch Sensortec's BSEC2 library, which Bosch licenses for use
with Bosch BME68x sensors and which may not be passed on to third parties. **This
repository does not contain or redistribute BSEC.** `tools/fetch_bsec.sh` (run by
setup) downloads the exact version used (v2.6.1.0) from Bosch's own GitHub and
asks you to accept Bosch's license terms first:
https://www.bosch-sensortec.com/media/boschsensortec/downloads/software/bme688_development_software/2024_12/20241219_clickthrough_license_terms_bsec_bme680_bme688_bme690.pdf

Bosch's BME68x Sensor API (BSD-3-Clause) is included in `third_party/bme68x/`.
