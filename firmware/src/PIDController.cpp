#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float outputLimit)
    : _kp(kp), _ki(ki), _kd(kd), _outputLimit(outputLimit) {}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

float PIDController::compute(float measured, float dtSeconds) {
    float error = _setpoint - measured;

    _integral += error * dtSeconds;
    // Clamp the integral term itself so a saturated output can't keep
    // accumulating (anti-windup), guarding against ki == 0.
    float integralLimit = (_ki > 0.0001f) ? (_outputLimit / _ki) : 0.0f;
    if (integralLimit > 0.0f) {
        if (_integral > integralLimit) _integral = integralLimit;
        if (_integral < -integralLimit) _integral = -integralLimit;
    }

    float derivative = 0.0f;
    if (!_firstCompute && dtSeconds > 0.0f) {
        derivative = (error - _prevError) / dtSeconds;
    }
    _firstCompute = false;
    _prevError = error;

    float output = _kp * error + _ki * _integral + _kd * derivative;

    if (output > _outputLimit) output = _outputLimit;
    if (output < -_outputLimit) output = -_outputLimit;

    return output;
}

void PIDController::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
    _firstCompute = true;
}
