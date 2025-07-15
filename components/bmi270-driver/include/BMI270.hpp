#pragma once
#include "I2C.hpp"
#include "IAccelSensor.hpp"
#include <stdint.h>

/// Driver for the Bosch BMI270 accelerometer over I2C
class BMI270 : public IAccelSensor {
public:
    /// Configuration parameters for BMI270
    struct Config {
        enum Range : uint8_t { RANGE_2G = 0, RANGE_4G, RANGE_8G, RANGE_16G };
        uint16_t odr_hz       = 100;      ///< Output data rate in Hz
        Range    accelRange   = RANGE_8G; ///< Full-scale accel range
        float    filterAlpha  = 0.2f;     ///< IIR filter α for accel smoothing
        uint8_t  i2c_address  = 0x68;     ///< Device I2C address
    };

    BMI270(I2C &bus, const Config &cfg);

    bool init() override;
    bool selfTest() override;
    bool dataReady() override;
    float getAccelX() override;
    float getAccelY() override;
    float getAccelZ() override;
    float getAccelLongitudinal() override;

    ~BMI270() override = default;

private:
    I2C   &_bus;
    Config _cfg;
    float  _lastFilteredX = 0.0f;
    float  _lastFilteredY = 0.0f;
    float  _lastFilteredZ = 0.0f;
    float  _scale = 0.0f; // g per LSB

    bool   writeRegister(uint8_t reg, uint8_t value);
    bool   readRegisters(uint8_t startReg, uint8_t *buf, size_t len);
    void   applyIIR(float &state, float raw);
};
