# DGB ICM45686 JSM

Development workspace for adapting the original DGB Dual Gyro Button firmware to two ICM-45686 IMUs on the RP2040, with DualShock 4 USB HID reporting for JoyShockMapper.

## Requirements

- RP2040 / Waveshare RP2040-Zero target
- Two ICM-45686 sensors on shared SPI0, separate chip-selects
- Preserve original DGB wiring: GP0 MISO, GP1 IMU 1 CS, GP2 SCK, GP3 MOSI, GP5 IMU 2 CS, GP9 touch pulse, GP12 touch input, 3V3 and GND
- USB HID: Sony DualShock 4 compatible report descriptor and 1 ms polling interval
- JoyShockMapper motion support
- Target 1000 Hz host reporting; sensor ODR must be configured and validated independently

## Bring-up and porting notes

The original DGB source README describes two BMI160 devices on SPI0. The ICM-45686 is not register-compatible with BMI160. Its driver must use the ICM-45686 register map and initialization sequence; BMI160 register writes must not be reused.

For each ICM-45686:
1. Keep its chip-select inactive while the other device is accessed.
2. Perform the ICM-45686 soft reset using the documented interface register.
3. Read WHO_AM_I (expected 0xE9).
4. Configure gyro low-noise mode and a supported ODR / full-scale range.
5. Verify data-ready and burst-read byte order before converting samples.
6. Confirm both sensors independently before enabling averaging/fusion.

The ICM-45686 supports up to 6.4 kHz ODR in low-noise mode; 1 kHz USB report cadence is a separate host-side requirement. Do not equate sensor ODR with USB HID polling.

## Current input artifact

DGB_ICM45686_Dual.uf2 was supplied for analysis. UF2 structure is valid (349 blocks, 89,344 bytes payload) and identifies RP2040 family ID 0xE48BFF56, flash target beginning at 0x10000000. This validates the target family, not sensor initialization or HID behavior.

## Status

- [x] Confirmed original DGB pinout from the source repository README.
- [x] Confirmed original firmware target is Waveshare RP2040-Zero / RP2040.
- [x] Inspected supplied UF2 container and target metadata.
- [ ] Obtain/read the current ICM45686 Arduino source project used to build the supplied UF2.
- [ ] Audit dual-sensor SPI transactions, chip-select behavior, ICM-45686 reset/WHO_AM_I/configuration and sample decoding.
- [ ] Audit USB HID descriptor and report format against DS4/JoyShockMapper expectations.
- [ ] Build reproducible UF2 and validate on physical hardware.

## Important limitation

The supplied UF2 is a compiled binary, not the source project. The destination repository was empty when inspected. A working firmware fix cannot be responsibly claimed until the source sketch/library configuration (or complete build project) is available and the hardware is tested.
