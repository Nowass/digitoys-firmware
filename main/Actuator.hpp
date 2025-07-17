#pragma once
#include "adas_pwm_driver.hpp"

class Actuator {
public:
    explicit Actuator(adas::PwmDriver& drv, size_t idx = 0)
        : _drv(drv), _idx(idx) {}

    void pausePassthrough() { _drv.pausePassthrough(_idx); }
    void resumePassthrough() { _drv.resumePassthrough(_idx); }
    void setBrakeDuty(float duty) { _drv.setDuty(_idx, duty); }
    float lastDuty() const { return _drv.lastDuty(_idx); }
    bool throttlePressed(float center = 0.09f, float range = 0.01f) const {
        return _drv.isThrottlePressed(_idx, center, range);
    }

private:
    adas::PwmDriver& _drv;
    size_t _idx;
};
