#pragma once
#include <Arduino.h>

// MPU9350 accel/gyro share the same register map as the MPU6050/MPU9250,
// so this talks to it directly over I2C without a dedicated library.
class IMU {
public:
    bool begin();

    // Robot must be held still and upright while this runs.
    void calibrate(uint16_t samples = 500);

    // Call every loop iteration; updates the filtered pitch estimate.
    void update();

    float getPitch() const { return _pitch; }

private:
    static constexpr uint8_t I2C_ADDRESS = 0x68;
    static constexpr float ACCEL_SENSITIVITY = 8192.0f;  // LSB/g at +-4g
    static constexpr float GYRO_SENSITIVITY = 65.5f;     // LSB/(deg/s) at +-500dps
    static constexpr float COMPLEMENTARY_ALPHA = 0.98f;

    void writeRegister(uint8_t reg, uint8_t value);
    void readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);

    float _pitch = 0.0f;
    float _gyroOffset = 0.0f;
    unsigned long _lastUpdateUs = 0;
};
