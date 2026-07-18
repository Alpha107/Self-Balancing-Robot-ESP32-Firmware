#include "MotorDriver.h"
#include <Arduino.h>
#include "Config.h"

void MotorDriver::begin() {
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_ENA, PWM_CHANNEL_LEFT);

    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_ENB, PWM_CHANNEL_RIGHT);

    stop();
}

void MotorDriver::setChannel(int pwmChannel, int in1Pin, int in2Pin, int speed) {
    bool forward = speed >= 0;
    int magnitude = constrain(abs(speed), 0, 255);

    digitalWrite(in1Pin, forward ? HIGH : LOW);
    digitalWrite(in2Pin, forward ? LOW : HIGH);
    ledcWrite(pwmChannel, magnitude);
}

void MotorDriver::setSpeed(int leftSpeed, int rightSpeed) {
    setChannel(PWM_CHANNEL_LEFT, PIN_IN1, PIN_IN2, leftSpeed);
    setChannel(PWM_CHANNEL_RIGHT, PIN_IN3, PIN_IN4, rightSpeed);
}

void MotorDriver::stop() {
    ledcWrite(PWM_CHANNEL_LEFT, 0);
    ledcWrite(PWM_CHANNEL_RIGHT, 0);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
}
