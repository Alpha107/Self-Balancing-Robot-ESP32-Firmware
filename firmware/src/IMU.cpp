#include "IMU.h"
#include <Wire.h>
#include "Config.h"

void IMU::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void IMU::readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    // ACCEL_XOUT_H (0x3B) .. GYRO_ZOUT_L (0x48): 14 contiguous bytes
    // (accel x/y/z, temp, gyro x/y/z), each a big-endian int16.
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)I2C_ADDRESS, (uint8_t)14, (uint8_t) true);

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // discard temperature
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
}

bool IMU::begin() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    writeRegister(0x6B, 0x00); // PWR_MGMT_1: wake from sleep, internal clock
    delay(50);
    writeRegister(0x1C, 0x08); // ACCEL_CONFIG: +-4g
    writeRegister(0x1B, 0x08); // GYRO_CONFIG: +-500 dps

    // Confirm the device responds (WHO_AM_I at 0x75).
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(0x75);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)I2C_ADDRESS, (uint8_t)1, (uint8_t) true);
    if (Wire.available() != 1) return false;
    Wire.read(); // don't hard-check the exact ID; MPU9350/9250/6050 vary by silicon rev

    _lastUpdateUs = micros();
    return true;
}

void IMU::calibrate(uint16_t samples) {
    float gyroSum = 0.0f;
    int16_t ax, ay, az, gx, gy, gz;

    for (uint16_t i = 0; i < samples; i++) {
        readRaw(ax, ay, az, gx, gy, gz);
        gyroSum += gy;
        delay(2);
    }
    _gyroOffset = gyroSum / samples;

    // Seed pitch from the accelerometer so we don't ramp up from 0.
    readRaw(ax, ay, az, gx, gy, gz);
    float accelPitch = atan2f(-ax, sqrtf((float)ay * ay + (float)az * az)) * RAD_TO_DEG;
    _pitch = accelPitch * IMU_INVERT_PITCH;
    _lastUpdateUs = micros();
}

void IMU::update() {
    unsigned long nowUs = micros();
    float dt = (nowUs - _lastUpdateUs) / 1000000.0f;
    _lastUpdateUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) return; // guard against overflow/startup glitches

    int16_t ax, ay, az, gx, gy, gz;
    readRaw(ax, ay, az, gx, gy, gz);

    float accelPitch = atan2f(-ax, sqrtf((float)ay * ay + (float)az * az)) * RAD_TO_DEG;
    float gyroRate = (gy - _gyroOffset) / GYRO_SENSITIVITY;

    accelPitch *= IMU_INVERT_PITCH;
    gyroRate *= IMU_INVERT_PITCH;

    _pitch = COMPLEMENTARY_ALPHA * (_pitch + gyroRate * dt)
           + (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;
}
