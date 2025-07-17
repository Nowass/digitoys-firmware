#pragma once
#include "I2C.hpp"
#include "BMI270.hpp"
#include "LiDAR.hpp"
#include "Actuator.hpp"
#include "PID.hpp"
#include "Monitor.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include <cmath>
#include <limits>
#include <algorithm>

struct FrontAssistConfig {
    static constexpr float D_WARNING     = 5.0f;
    static constexpr float D_BRAKE       = 1.5f;
    static constexpr float A_MAX         = 2.5f;
    static constexpr float FILTER_ALPHA  = 0.2f;
    static constexpr float FULL_BRAKE_DUTY = 1.0f;
    static constexpr TickType_t TICK_PERIOD = pdMS_TO_TICKS(50);
};

class ControlTask {
public:
    ControlTask(I2C& i2cBus, lidar::LiDAR& lidar, Actuator& driver, monitor::Monitor* mon)
        : _accel(i2cBus, {.filterAlpha = FrontAssistConfig::FILTER_ALPHA})
        , _lidar(lidar)
        , _driver(driver)
        , _pid(1.0f, 0.0f, 0.1f)
        , _mon(mon) {}

    void run() {
        _accel.init();
        _accel.selfTest();
        _driver.resumePassthrough();
        _lastDistance = std::numeric_limits<float>::infinity();

        while (true) {
            float d = _lidar.getDistance();

            switch (_state) {
            case State::Idle:    handleIdle(d);    break;
            case State::Warning: handleWarning(d); break;
            case State::Braking: handleBraking(d); break;
            }

            _lastDistance = d;
            if (_mon) {
                bool obstacle = d <= FrontAssistConfig::D_BRAKE;
                bool warning = !obstacle && d < FrontAssistConfig::D_WARNING;
                _mon->updateTelemetry(obstacle, d, _vEst, warning);
            }
            vTaskDelay(FrontAssistConfig::TICK_PERIOD);
        }
    }

private:
    enum class State { Idle, Warning, Braking };
    State _state = State::Idle;

    BMI270    _accel;
    lidar::LiDAR& _lidar;
    Actuator& _driver;
    PID       _pid;
    monitor::Monitor* _mon;
    float     _aFiltered = 0.0f;
    float     _vEst = 0.0f;
    float     _lastDistance = 0.0f;

    void handleIdle(float d) {
        if (d < FrontAssistConfig::D_WARNING) {
            _state = State::Warning;
        }
    }

    void handleWarning(float d) {
        if (d <= FrontAssistConfig::D_BRAKE) {
            _state = State::Braking;
            return;
        }
        if (d >= FrontAssistConfig::D_WARNING) {
            _driver.resumePassthrough();
            _state = State::Idle;
            return;
        }

        if (_accel.dataReady()) {
            float aRaw = _accel.getAccelLongitudinal();
            _aFiltered = FrontAssistConfig::FILTER_ALPHA * _aFiltered +
                         (1.0f - FrontAssistConfig::FILTER_ALPHA) * aRaw;
            _vEst = std::max(0.0f, _vEst +
                             _aFiltered * (FrontAssistConfig::TICK_PERIOD / 1000.0f));
        }

        float dRem = std::max(d - FrontAssistConfig::D_BRAKE, 0.01f);
        float aReq = std::min((_vEst * _vEst) / (2.0f * dRem), FrontAssistConfig::A_MAX);

        if (d >= _lastDistance)
            aReq = 0.0f;

        float error = -aReq - _aFiltered;
        float brakeCmd = _pid.update(error, FrontAssistConfig::TICK_PERIOD / 1000.0f);

        _driver.pausePassthrough();
        _driver.setBrakeDuty(std::clamp(brakeCmd, 0.0f, FrontAssistConfig::FULL_BRAKE_DUTY));
    }

    void handleBraking(float d) {
        if (d >= FrontAssistConfig::D_WARNING) {
            _driver.resumePassthrough();
            _state = State::Idle;
        } else if (d > FrontAssistConfig::D_BRAKE) {
            _state = State::Warning;
        } else {
            _driver.pausePassthrough();
            _driver.setBrakeDuty(FrontAssistConfig::FULL_BRAKE_DUTY);
        }
    }
};
