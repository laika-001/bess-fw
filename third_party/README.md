# Third-party code

| Folder | What | Version | License |
|---|---|---|---|
| `bme68x/` | Bosch BME68x Sensor API (BME688 driver), from github.com/boschsensortec/BME68x_SensorAPI | v4.4.8 (commit 80ea120) | BSD-3-Clause, see `bme68x/LICENSE` |
| `bsec2/` | Bosch BSEC2 (IAQ, CO2-eq, bVOC-eq), from github.com/boschsensortec/Bosch-BSEC2-Library | library 1.10.2610 / BSEC 2.6.1.0 (commit 4f559a6) | Bosch Sensortec license. **Not redistributed here**: `tools/fetch_bsec.sh` downloads it and asks you to accept Bosch's terms; see `bsec2/README.md` |

The BME68x driver is copied here unchanged (only the files the firmware uses).
BSEC is not: see `bsec2/README.md` for what `tools/fetch_bsec.sh` downloads.
