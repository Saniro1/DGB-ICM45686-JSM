# DGB ICM45686 JSM

Development workspace for adapting the original DGB Dual Gyro Button firmware to two ICM-45686 IMUs on the RP2040, preserving the original DGB wiring and JoyShockMapper-oriented USB behavior.

## Hardware constraints

- Board: Waveshare RP2040-Zero / RP2040
- Two ICM-45686 sensors share SPI0 with independent chip selects
- GP0 MISO, GP1 IMU 1 CS, GP2 SCK, GP3 MOSI, GP5 IMU 2 CS
- GP9 touch pulse, GP12 touch input
- Preserve the original 3V3/GND arrangement

## Confirmed ICM-45686 facts

- WHO_AM_I is 0xE9.
- INT1_STATUS0 RESET_DONE is bit 7 (0x80); the uploaded source's bit value was correct.
- SREG_CTRL is indirect register 0xA267; reset default selects little-endian sensor data.
- IREG address setup at 0x7C/0x7D followed by reading 0x7E is consistent with TDK's transport driver. The uploaded source's burst-address sequence was not the problem.
- ODR code 0x5 selects 1600 Hz for both accel and gyro.
- Full-scale code 0x1 selects +/-2000 dps gyro and +/-16 g accel.
- TDK's Arduino reference transport uses SPI mode 3 at 6 MHz by default; the datasheet supports mode 0 or 3 depending on the interface configuration.

References:
- TDK ICM-45686 datasheet: https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf
- TDK Arduino driver: https://github.com/tdk-invn-oss/motion.arduino.ICM45686

## Current work

Branch: `fix/tdk-verified-icm45686`

Patch: [patches/icm45686-driver-fix.patch](patches/icm45686-driver-fix.patch)

The patch:
1. Aligns SPI mode and initial clock with TDK's Arduino reference.
2. Preserves DRIVE_CONFIG0 and INTF_CONFIG1_OVRD across soft reset, following TDK's reset routine.
3. Keeps the correct RESET_DONE bit and IREG access behavior.
4. Prevents failed/uninitialized IMUs from being read, calibrated, or averaged as valid sensor data.
5. Allows one valid sensor to be used temporarily for diagnosis if the other sensor fails.

The patch was checked to apply cleanly to the supplied ZIP source. All source translation units passed a host-side C++ syntax check using lightweight Arduino/SPI stubs. This is not a full Arduino-core build and does not prove the firmware works on the RP2040.

## Remaining validation

- Audit and reconcile the inherited USB HID descriptor/report layout with the actual report bytes and the reference UF2.
- Verify USB VID/PID and Windows enumeration, then JoyShockMapper detection.
- Build with the exact RP2040 Arduino core and Adafruit TinyUSB versions.
- Test both sensors independently, then dual-sensor fusion, touch gating, and 1000 Hz host reporting on hardware.

**Do not flash this patch as a verified UF2 yet.** It is the first source-level bring-up correction, not a hardware-tested release. Sensor ODR and USB report cadence are separate; 1600 Hz sensor ODR does not itself guarantee 1000 Hz USB reports.
