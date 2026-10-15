# BESS firmware: setup

Firmware for the MAX32630FTHR: WDT0 watchdog plus BME688, TMP117 and ABP2 sensors on I2C1.
Built with mbed-os (CMake + GCC ARM), flashed with pyocd over the board's DAPLink USB.

## Setup

You need: Homebrew (https://brew.sh) and the Xcode Command Line Tools
(`xcode-select --install`); the MAX32630FTHR with a MAX32625PICO debug adapter
(10-pin SWD ribbon) and its own USB cable for power; the sensors wired as in
"Hardware" below.

1. Clone and enter the repository:

   ```bash
   git clone https://github.com/laika-001/bess-fw.git
   cd bess-fw
   ```

2. Run the one-time setup. It is safe to re-run.

   ```bash
   tools/setup_mac.sh
   ```

   It installs the Homebrew toolchain (asks for your password once, for the ARM
   compiler), creates the Python venv at `~/.venvs/mbed`, installs the Python
   packages from `tools/wheels/` offline, clones mbed-os at the commit pinned in
   `mbed-os.lib`, **downloads Bosch BSEC2** (see below; it asks you to accept
   Bosch's license), and adds the `bessflash` / `bessmon` commands to `~/.zshrc`.

3. Open a new terminal, plug in the adapter and the board, and run:

   ```bash
   bessflash
   ```

4. Watch the output with `bessmon` (exit with Ctrl-A then K). After the boot
   messages you should see one reading line and one `$BESS,3,...` frame every 3 s.

**Bosch BSEC2 is not in this repository.** Bosch's license allows using the BSEC
binary in products with Bosch BME68x sensors but not passing it on, so this
repository does not redistribute it. `tools/fetch_bsec.sh` downloads the exact
version the firmware was built with (BSEC v2.6.1.0, Bosch-BSEC2-Library commit
4f559a6) from Bosch's GitHub into `third_party/bsec2/`, checks the SHA-256 of every
file, and asks you to accept Bosch's terms first. Setup runs it; to run it on its
own: `tools/fetch_bsec.sh` (or `--accept-bosch-license` to skip the prompt).
Bosch's open-source BME68x driver (BSD-3-Clause) is included in `third_party/bme68x/`.

**Raspberry Pi dashboard and detection rules:** see `pi/README.md` (gateway wiring,
service, deploy) and `DETECTION.md` (what the module checks and why).

### Moving an existing checkout

Copy the folder (you can leave out `cmake_build/` and the ~860 MB `mbed-os/`), or
clone again, then run step 2 above.

Requires Homebrew (https://brew.sh) and the Xcode Command Line Tools (`xcode-select --install`).

**If the ARM compiler step fails with "Cask reports different checksum":** ARM
re-published the installer and Homebrew's cask hasn't caught up. Download the
`.pkg` named in the error from ARM, compare its SHA-256 with ARM's `.sha256asc`
file next to it, then install it and re-run setup. Setup skips the cask once
`/Applications/ArmGNUToolchain/<version>/arm-none-eabi/bin` has a compiler.

```bash
sudo installer -pkg arm-gnu-toolchain-<version>-darwin-arm64-arm-none-eabi.pkg -target /
```

If that installer sits silent for minutes, unplug the board: the OS checks every
mounted drive first and can hang on the board's DAPLINK drive.

## Hardware

| Part | Connection | I2C address |
|---|---|---|
| MAX32630FTHR | main board; its own USB cable powers it | |
| MAX32625PICO | debug adapter: USB to the computer, 10-pin SWD ribbon to the FTHR, stripe on pin 1 at both ends | |
| Adafruit BME688 (STEMMA QT, #5046) | I2C1: SDA (blue, "SDI") to P3_4, SCL (yellow, "SCK") to P3_5, VIN (red) to **3V3**, GND | 0x77 |
| TMP117 | same I2C1 bus, 3V3, GND | 0x48 |
| Honeywell ABP2DANT001PG2A3XX (bare DIP) | same I2C1 bus; pin 1 GND, 2 VDD (3V3), 5 SDA, 6 SCL | 0x28 |
| Raspberry Pi 5 (optional gateway) | MAX P3_1 (TX) to Pi pin 10, MAX P3_0 (RX) to Pi pin 8, GND to GND | |

Power every sensor from 3V3, never VBUS (see "Board gotchas"). A missing sensor
makes the board reset itself every ~5.6 s until it answers.

## Daily commands

| Command | What it does |
|---|---|
| `bessflash` | Build and flash (`tools/bessflash.sh`) |
| `bessflash --build-only` | Build without flashing |
| `bessmon` | Serial console at 115200 (`tools/monitor.sh`); exit with Ctrl-A then K |
| `tools/clear_pmic_bus.sh` | Recover a silent board (see below) |
| `~/.venvs/mbed/bin/python tools/bess_rx.py --log-dir logs` | Log the `$BESS` frames: one CSV + raw text file per board boot in `logs/`. Keeps running through reflashes and replugs. Prefix with `caffeinate -i` for overnight runs |
| `~/.venvs/mbed/bin/python tools/bess_rx.py --csv log.csv` | Same, into a single CSV |

The serial port name (`/dev/cu.usbmodemXXXXX`) changes between machines and USB
ports; the scripts find it automatically.

## What's in `tools/`

| File | Purpose |
|---|---|
| `setup_mac.sh` | One-time setup, described above |
| `Brewfile` | `cmake`, `ninja`, `gcc-arm-embedded` |
| `requirements.txt` | Pinned Python packages (mbed-tools 7.59.0, pyocd 0.45.1 and dependencies) |
| `wheels/` | Those packages as offline wheels for arm64 + Python 3.9. On a different Python, setup falls back to PyPI with the same pins. |
| `env.sh` | Shared paths used by the other scripts |
| `bessflash.sh`, `monitor.sh`, `clear_pmic_bus.sh` | The commands above |
| `fetch_bsec.sh` | Downloads Bosch BSEC2 (not redistributed here) and checks its checksums |
| `bess_rx.py` | Receives, checks and logs the `$BESS` frames |
| `check_rules.py` | Replays logged frames through `rules.h` to show what each threshold would have done |
| `rules_json.py` | Extracts the thresholds from `rules.h` for the Pi dashboard (run by `pi/deploy.sh`) |

## Board gotchas (learned the hard way)

- **The header 3V3 pin is off at power-up.** It comes from the PMIC's LDO3, which plain
  mbed-os never enables. `power_check()` in `main.cpp` turns it on at every boot.
  Power all sensors from 3V3, never VBUS: the breakouts' I2C pull-ups go to their VIN,
  so VIN on VBUS puts the whole bus at 5 V.
- **The ABP2 needs the I2C pins on the 3.3 V I/O rail.** P3_4/P3_5 default to 1.8 V.
  The Adafruit breakouts have level shifters and don't care; the bare ABP2 doesn't
  respond until `MXC_IOMAN->use_vddioh_0` bits 28/29 are set (done in `main()`).
- **Breakout labels:** on the BME688 board, SCK = I2C SCL and SDI = I2C SDA.
- **ABP2 DIP pinout:** 1 GND, 2 VDD, 3 EOC, 4 NC, 5 SDA, 6 SCL (not the old ABP layout).
- **Silent board (no LED, no serial):** an on-board chip is holding the I2C2 data line
  low, and the startup code waits forever on it before `main()`. Run
  `tools/clear_pmic_bus.sh` or unplug USB.
- **Missing sensor = reboot loop, by design.** If any sensor doesn't answer, the firmware
  stops feeding the watchdog and the board resets every ~5.6 s until it does.
- **`bessflash` builds but then says "SWD/JTAG communication failure (No ACK)":** the
  MAX32625PICO adapter can't reach the MAX32630 over the 10-pin ribbon. The serial
  console can still work (it uses a different ribbon wire), so a live console does
  not prove the debug wires are connected. Reseat the ribbon at both ends with the
  stripe on pin 1 and all ten pins in, or swap the ribbon. To confirm:
  copy `bess-blinky.bin` onto the DAPLINK drive; if `FAIL.TXT` then says
  "FAILED to reset/halt the target MCU", the adapter can't drive the board's
  SWD wires at all. Pin 1 of the ribbon (target voltage sense, red stripe) is the
  usual culprit: the PICO's DAPLink firmware routes SWDIO/SWCLK through an analog
  switch and takes its I/O level from that pin, while reset and the serial console
  bypass it. So serial and reset can work with SWD completely dead. Never hook
  the PICO up with jumper wires that leave pin 1 out.
- **Build output:** mbed-tools prints `FAILED: bess-blinky.elf` from its memory-report
  step even on success; `bessflash.sh` checks for the `-- built:` line instead.

## Data protocol (v4)

Every 3 s (the cycle is timed by BSEC) the firmware sends one line on the USB serial
port, after the debug text:

```
$BESS,4,<seq>,<up_ms>,<flags>,<t117_cC>,<dp_cPa>,<tabp_cC>,<tbme_cC>,<p_Pa>,<rh_cP>,<gas_ohm>,<heat_c>,<iaq_c>,<iaq_acc>,<siaq_c>,<co2eq_cppm>,<bvoc_ppb>,<bsec_st>,<rules>*<CRC16>
```

Integer fixed-point values (cC = 0.01 °C, cPa = 0.01 Pa, cP = 0.01 %RH), empty when
a sensor read failed, CRC-16/CCITT-FALSE over the bytes between `$` and `*`. The
frame doubles as the heartbeat: no valid frame for 6 s = fault. Field and flag
definitions are in `bess_proto.h`; `tools/bess_rx.py` is the reference receiver and
also reads the older versions (v1 ends at `gas_ohm`, v2 at `bsec_st`, v3 carried a
since-removed autoencoder score). `rules` is 4 hex digits of the module's threshold
results; flag 0x0020 means possible battery event. See `DETECTION.md`.

## BME688 and BSEC

The BME688 runs through Bosch's BME68x driver and Bosch's BSEC2 library
(`bme_bsec.cpp`, sources and licenses in `third_party/`). BSEC adds an IAQ index,
its accuracy, a static IAQ, a CO2 equivalent and a breath-VOC equivalent.

- **These are estimates from the single gas resistance.** "CO2 equivalent" is inferred
  from VOCs; the BME688 cannot sense CO2. Near batteries, electrolyte vapour raises it.
- **Accuracy (`iaq_acc`) starts at 0** and climbs to 3 as BSEC calibrates, over hours
  to days. Until then IAQ reads 50 and CO2-eq 500 ppm.
- **Heater:** after every boot BSEC runs a 7-cycle heater run-in (380 °C x4, 150 °C,
  200 °C x2), then holds 320 °C for 197 ms. `heat_c` says which; for gas analysis
  keep rows with `heat_c` = 320 and drop each boot's first 30 minutes.
- **Calibration is saved in flash** (last page, 0x1FE000) the first time accuracy
  reaches 3 after a boot, then every 6 hours, and loaded at boot (`BSEC state: loaded`
  in the boot messages). Reflashing doesn't erase it. A different BME688 should start
  fresh: erase that page with `pyocd erase -t max32630 -s 0x1FE000+0x2000`.
- **Config:** `bme688_sel_33v_3s_4d` (BME688, 3.3 V, 3 s, 4-day history), set by
  `BESS_BSEC_SEL_CONFIG` in `bme_bsec.cpp`.

## ABP2 zero offset

The ABP2 reads about 22.7 Pa with its port open to air. That is inside Honeywell's
spec, which assumes you subtract it ("auto-zero"). `ABP2_ZERO_COUNTS` in `main.cpp`
does that, so `dp` in the frames is true gauge pressure. Re-measure it after
re-soldering, re-mounting or replacing the sensor, and now and then in service:

1. Leave the port open to air, nothing connected, and log a few minutes:
   `~/.venvs/mbed/bin/python tools/bess_rx.py --csv zero.csv` (Ctrl-C to stop).
2. Print the corrected constant (the current value is read from `main.cpp`):

   ```bash
   python3 -c "import csv,re,statistics as s; d=[float(r['dp_Pa']) for r in csv.DictReader(open('zero.csv')) if r['dp_Pa']]; z=int(re.search(r'ABP2_ZERO_COUNTS = (-?\d+)',open('main.cpp').read()).group(1)); print(f'mean {s.mean(d):.2f} Pa over {len(d)} readings -> ABP2_ZERO_COUNTS = {z+round(s.mean(d)/6894.757*13421772)}')"
   ```
3. Put that number in `main.cpp` and run `bessflash`.

## Source files

- `main.cpp`: the firmware that gets built.
- `bess_proto.h`: the data frame encoder (no mbed dependencies).
- `bme_bsec.cpp`, `bme_bsec.h`: BME688 through Bosch's driver and BSEC2, with the flash-saved state.
- `rules.h`: the on-board threshold checks and the thresholds themselves (no mbed code);
  see `DETECTION.md`. `tools/check_rules.py` replays logs through it.
- `pi/`: the Raspberry Pi gateway dashboard (UART link, service, nginx, deploy script); see `pi/README.md`.
- `third_party/`: Bosch BME68x driver and BSEC2 files, with their licenses (see its README).
- `main_hand_bme688.cpp`: main.cpp before BSEC, with the hand-written BME688 code.
- `main_sensors_v1.cpp`: main.cpp before the data protocol was added (copy back over
  main.cpp to revert).
- `main_wdt_test.cpp`, `bme688_full_test.cpp`, `abp2_only_test.cpp`: earlier standalone
  tests, kept for reference and not built. To run one, copy it over `main.cpp`.
