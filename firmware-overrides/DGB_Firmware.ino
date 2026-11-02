#include "config.h"
#include "icm45686_spi.h"
#include "touch_sensor.h"
#include "usb_ds4_hid.h"

static DS4InputReport ds4Report;
static uint32_t lastReportUs = 0;
static uint16_t reportCounter = 0;

// First-order low-pass filters. Lower alpha means stronger smoothing.
static float filteredGyroX = 0.0f;
static float filteredGyroY = 0.0f;
static float filteredGyroZ = 0.0f;
static float filteredAccelX = 0.0f;
static float filteredAccelY = 0.0f;
static float filteredAccelZ = 0.0f;
static bool motionFilterInitialized = false;

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
  if ((uint32_t)(now - lastReportUs) < 1000) {
    return;
  }
  lastReportUs = now;
  ++reportCounter;

  const bool touched = touchSensor.readTouch();
  ImuRawData motion;
  imu.readFusedMotion(motion);

  // Reverse Y and Z at the sensor-to-report mapping boundary.
  const float targetGyroX = (float)motion.gx;
  const float targetGyroY = -((float)motion.gz);
  const float targetGyroZ = (float)motion.gy;

  // Seed all filters from the first real sample to avoid a startup ramp.
  if (!motionFilterInitialized) {
    filteredGyroX = targetGyroX;
    filteredGyroY = targetGyroY;
    filteredGyroZ = targetGyroZ;
    filteredAccelX = (float)motion.ax;
    filteredAccelY = (float)motion.ay;
    filteredAccelZ = (float)motion.az;
    motionFilterInitialized = true;
  } else {
    // Keep pitch (X) and yaw (Y) responsive; smooth roll (Z) more strongly.
    filteredGyroX += 0.40f * (targetGyroX - filteredGyroX);
    filteredGyroY += 0.40f * (targetGyroY - filteredGyroY);
    filteredGyroZ += 0.20f * (targetGyroZ - filteredGyroZ);

    // Light smoothing on all accelerometer axes.
    filteredAccelX += 0.25f * ((float)motion.ax - filteredAccelX);
    filteredAccelY += 0.25f * ((float)motion.ay - filteredAccelY);
    filteredAccelZ += 0.25f * ((float)motion.az - filteredAccelZ);
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
