#pragma once

/// Abstract interface for any 6-axis IMU sensor (accelerometer + gyroscope)
class IIMUSensor
{
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

    /// Read raw gyroscope components in degrees/s
    virtual float getGyroX() = 0;
    virtual float getGyroY() = 0;
    virtual float getGyroZ() = 0;

    /// Read temperature in degrees Celsius
    virtual float getTemperature() = 0;

    /// Convenience: return the longitudinal (forward) accel in m/s²
    virtual float getAccelLongitudinal() = 0;

    virtual ~IIMUSensor() = default;
};
