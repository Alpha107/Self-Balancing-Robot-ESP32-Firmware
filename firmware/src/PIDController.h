#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outputLimit);

    void setGains(float kp, float ki, float kd);
    void setSetpoint(float setpoint) { _setpoint = setpoint; }

    // measured: current angle (deg). dtSeconds: time since last call.
    float compute(float measured, float dtSeconds);

    void reset();

    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

private:
    float _kp;
    float _ki;
    float _kd;
    float _outputLimit;

    float _setpoint = 0.0f;
    float _integral = 0.0f;
    float _prevError = 0.0f;
    bool _firstCompute = true;
};
