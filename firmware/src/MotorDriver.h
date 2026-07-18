#pragma once

class MotorDriver {
public:
    void begin();

    // speed range: -255 (full reverse) .. 255 (full forward)
    void setSpeed(int leftSpeed, int rightSpeed);
    void stop();

private:
    void setChannel(int pwmChannel, int in1Pin, int in2Pin, int speed);
};
