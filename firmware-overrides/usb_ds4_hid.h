#ifndef USB_DS4_HID_H_
#define USB_DS4_HID_H_
#include "config.h"
#include <Adafruit_TinyUSB.h>
struct DS4InputReport {
 uint8_t lx,ly,rx,ry;
 uint8_t buttons1,buttons2,buttons3;
 uint8_t l2_trigger,r2_trigger;
 uint16_t timestamp;
 uint8_t battery;
 int16_t gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z;
 uint8_t touch_track_num,finger1_id,finger1_x_low,finger1_x_high_y_low,finger1_y_high;
 uint8_t reserved[34];
} __attribute__((packed));
static_assert(sizeof(DS4InputReport)==63,"DS4 HID payload must be 63 bytes");
class UsbDs4Hid { public: UsbDs4Hid(); void begin(); bool sendReport(const DS4InputReport&); bool isReady(); private: Adafruit_USBD_HID _usbHid; };
extern UsbDs4Hid usbDs4;
#endif
