#pragma once

class PID {
public:
    PID(float kp, float ki, float kd)
        : _kp(kp), _ki(ki), _kd(kd) {}

    float update(float error, float dt) {
        _integral += error * dt;
        float derivative = (error - _prevError) / dt;
        _prevError = error;
        return _kp * error + _ki * _integral + _kd * derivative;
    }

    void reset() {
        _integral = 0.0f;
        _prevError = 0.0f;
    }

private:
    float _kp;
    float _ki;
    float _kd;
    float _integral = 0.0f;
    float _prevError = 0.0f;
};
