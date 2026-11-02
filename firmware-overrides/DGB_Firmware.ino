#include "config.h"
#include "icm45686_spi.h"
#include "touch_sensor.h"
#include "usb_ds4_hid.h"

static DS4InputReport ds4Report;
static uint32_t lastReportUs = 0;
static uint16_t reportCounter = 0;

// Aggressive gyro drift suppression for the stationary test build.
static float filteredGyroX = 0.0f;
static float filteredGyroY = 0.0f;
static float filteredGyroZ = 0.0f;
static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;
static bool gyroBiasInitialized = false;
static float filteredAccelX = 0.0f;
static float filteredAccelY = 0.0f;
static float filteredAccelZ = 0.0f;
static bool motionFilterInitialized = false;

// Three-sample median rejects isolated one-sample spikes without averaging
// across the full movement signal. Applied only to roll and accelerometer.
static float previousGyroZ1 = 0.0f;
static float previousGyroZ2 = 0.0f;
static float previousAccelX1 = 0.0f, previousAccelX2 = 0.0f;
static float previousAccelY1 = 0.0f, previousAccelY2 = 0.0f;
static float previousAccelZ1 = 0.0f, previousAccelZ2 = 0.0f;
static uint8_t spikeFilterSamples = 0;

static float medianOfThree(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

void setup() {
  Serial.begin(115200);
  touchSensor.begin();
  usbDs4.begin();

  bool imuOk = imu.begin();
  Serial.println("Dual ICM-45686 DGB");
  Serial.print("IMU1 WHO_AM_I=0x");
  Serial.print(imu.whoAmI(0), HEX);
  Serial.println(imu.sensorOk(0) ? " OK" : " FAIL");
  Serial.print("IMU2 WHO_AM_I=0x");
  Serial.print(imu.whoAmI(1), HEX);
  Serial.println(imu.sensorOk(1) ? " OK" : " FAIL");

  if (!imuOk) {
    Serial.println("Check 3V3/GND and SPI: GP0 MISO, GP1 CS1, GP2 SCK, GP3 MOSI, GP5 CS2");
  }

  Serial.println("Keep unit stationary: calibrating");
  imu.calibrateRestingOffsets();

  memset(&ds4Report, 0, sizeof(ds4Report));
  ds4Report.lx = ds4Report.ly = ds4Report.rx = ds4Report.ry = 128;
  ds4Report.buttons1 = 0x08;
  ds4Report.battery = 0x0B;
  lastReportUs = micros();
}

void loop() {
  uint32_t now = micros();
  // Pace DS4 reports at ~980 Hz (1020 us), slightly below the 1 ms USB interval.
  if ((uint32_t)(now - lastReportUs) < 1020) {
    return;
  }
  lastReportUs = now;
  ++reportCounter;

  const bool touched = touchSensor.readTouch();
  ImuRawData motion;
  imu.readFusedMotion(motion);

  // Reverse Y and Z at the sensor-to-report mapping boundary.
  const float rawGyroX = (float)motion.gx;
  const float rawGyroY = -((float)motion.gz);
  const float rawGyroZ = (float)motion.gy;
  const float rawAccelX = (float)motion.ax;
  const float rawAccelY = (float)motion.ay;
  const float rawAccelZ = (float)motion.az;

  // During startup, seed the history. Once three samples are available,
  // median filtering rejects isolated spikes on roll and all accel axes.
  float targetGyroZ = rawGyroZ;
  float targetAccelX = rawAccelX;
  float targetAccelY = rawAccelY;
  float targetAccelZ = rawAccelZ;
  if (spikeFilterSamples >= 2) {
    targetGyroZ = medianOfThree(previousGyroZ2, previousGyroZ1, rawGyroZ);
    targetAccelX = medianOfThree(previousAccelX2, previousAccelX1, rawAccelX);
    targetAccelY = medianOfThree(previousAccelY2, previousAccelY1, rawAccelY);
    targetAccelZ = medianOfThree(previousAccelZ2, previousAccelZ1, rawAccelZ);
  }
  previousGyroZ2 = previousGyroZ1;
  previousGyroZ1 = rawGyroZ;
  previousAccelX2 = previousAccelX1;
  previousAccelX1 = rawAccelX;
  previousAccelY2 = previousAccelY1;
  previousAccelY1 = rawAccelY;
  previousAccelZ2 = previousAccelZ1;
  previousAccelZ1 = rawAccelZ;
  if (spikeFilterSamples < 2) ++spikeFilterSamples;

  // Learn the residual zero-rate bias while the gyro vector is quiet.
  // This intentionally favors stillness over responsiveness for this test.
  if (!gyroBiasInitialized) {
    gyroBiasX = rawGyroX;
    gyroBiasY = rawGyroY;
    gyroBiasZ = rawGyroZ;
    gyroBiasInitialized = true;
  }
  const float gyroMagnitudeSq =
      rawGyroX * rawGyroX + rawGyroY * rawGyroY + rawGyroZ * rawGyroZ;
  if (gyroMagnitudeSq < (400.0f * 400.0f)) {
    const float biasAlpha = 0.005f;
    gyroBiasX += biasAlpha * (rawGyroX - gyroBiasX);
    gyroBiasY += biasAlpha * (rawGyroY - gyroBiasY);
    gyroBiasZ += biasAlpha * (rawGyroZ - gyroBiasZ);
  }

  // Remove the learned bias, then apply a large zero-rate deadband.
  float targetGyroX = rawGyroX - gyroBiasX;
  float targetGyroY = rawGyroY - gyroBiasY;
  float targetGyroZCorrected = rawGyroZ - gyroBiasZ;
  const float gyroDeadband = 18.0f;
  if (fabsf(targetGyroX) < gyroDeadband) targetGyroX = 0.0f;
  if (fabsf(targetGyroY) < gyroDeadband) targetGyroY = 0.0f;
  if (fabsf(targetGyroZCorrected) < gyroDeadband) targetGyroZCorrected = 0.0f;

  // Seed all filters from the first corrected sample to avoid a startup ramp.
  if (!motionFilterInitialized) {
    filteredGyroX = targetGyroX;
    filteredGyroY = targetGyroY;
    filteredGyroZ = targetGyroZCorrected;
    filteredAccelX = targetAccelX;
    filteredAccelY = targetAccelY;
    filteredAccelZ = targetAccelZ;
    motionFilterInitialized = true;
  } else {
    // Very strong smoothing on all gyro axes. Responsiveness is intentionally
    // sacrificed in this experiment to suppress residual jitter and drift.
    const float gyroFilterAlpha = 0.025f;
    filteredGyroX += gyroFilterAlpha * (targetGyroX - filteredGyroX);
    filteredGyroY += gyroFilterAlpha * (targetGyroY - filteredGyroY);
    filteredGyroZ += gyroFilterAlpha * (targetGyroZCorrected - filteredGyroZ);

    // Light smoothing on all accelerometer axes.
    filteredAccelX += 0.25f * (targetAccelX - filteredAccelX);
    filteredAccelY += 0.25f * (targetAccelY - filteredAccelY);
    filteredAccelZ += 0.25f * (targetAccelZ - filteredAccelZ);
  }

  // Y and Z were already inverted above; do not invert them again at HID output.
  ds4Report.gyro_x = (int16_t)(filteredGyroX >= 0.0f ? filteredGyroX + 0.5f : filteredGyroX - 0.5f);
  ds4Report.gyro_y = (int16_t)(filteredGyroY >= 0.0f ? filteredGyroY + 0.5f : filteredGyroY - 0.5f);
  int32_t reportGyroZ = (int32_t)(filteredGyroZ >= 0.0f ? filteredGyroZ + 0.5f : filteredGyroZ - 0.5f);
  if (reportGyroZ > 32767) reportGyroZ = 32767;
  if (reportGyroZ < -32768) reportGyroZ = -32768;
  ds4Report.gyro_z = (int16_t)reportGyroZ;

  ds4Report.accel_x = (int16_t)(filteredAccelX >= 0.0f ? filteredAccelX + 0.5f : filteredAccelX - 0.5f);
  ds4Report.accel_y = (int16_t)(filteredAccelY >= 0.0f ? filteredAccelY + 0.5f : filteredAccelY - 0.5f);
  ds4Report.accel_z = (int16_t)(filteredAccelZ >= 0.0f ? filteredAccelZ + 0.5f : filteredAccelZ - 0.5f);

  // Keep touch state independent from motion reporting.
  ds4Report.finger1_id = (uint8_t)((touched ? 0x00 : 0x80) | (reportCounter & 0x7F));
  if (touched) {
    ds4Report.buttons3 |= 0x02;
  } else {
    ds4Report.buttons3 &= (uint8_t)~0x02;
  }

  ds4Report.timestamp = (uint16_t)now;
  ds4Report.touch_track_num = (uint8_t)reportCounter;
  usbDs4.sendReport(ds4Report);
}
