#pragma once

/// Abstract interface for any 3-axis accelerometer sensor
class IAccelSensor {
public:
    /// Initialize the sensor; return true on success
    virtual bool init() = 0;

    /// Perform any self-test / calibration routines (optional)
    virtual bool selfTest() = 0;

    /// Return true if new data is available (DRDY flag or FIFO)
    virtual bool dataReady() = 0;

    /// Read raw acceleration components in m/s²
    virtual float getAccelX() = 0;
    virtual float getAccelY() = 0;
    virtual float getAccelZ() = 0;

    /// Convenience: return the longitudinal (forward) accel in m/s²
    virtual float getAccelLongitudinal() = 0;

    virtual ~IAccelSensor() = default;
};
