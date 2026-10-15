# Bosch BSEC2: not included

This folder is filled by `tools/fetch_bsec.sh` (setup runs it). Bosch's BSEC2 is
licensed by Bosch Sensortec for use with Bosch BME68x sensors and may not be
passed on to third parties, so this repository does not redistribute it.

The script downloads, from Bosch's GitHub (boschsensortec/Bosch-BSEC2-Library,
commit 4f559a6, BSEC v2.6.1.0), and checks the SHA-256 of:

- `inc/bsec_datatypes.h`, `inc/bsec_interface.h`: the BSEC headers
- `cortex-m4/libalgobsec.a`: the soft-float Cortex-M4 build. The `fpv4-sp-d16-hard`
  build passes floats in FPU registers, which doesn't link with mbed's
  `-mfloat-abi=softfp`. Bosch marks the M4 builds "not tested".
- `config/bme688_sel_33v_3s_4d/bsec_selectivity.txt`: BME688, 3.3 V, 3 s (LP), 4-day history
- `LICENSE.md`: Bosch's license notice

License terms: https://www.bosch-sensortec.com/media/boschsensortec/downloads/software/bme688_development_software/2024_12/20241219_clickthrough_license_terms_bsec_bme680_bme688_bme690.pdf
