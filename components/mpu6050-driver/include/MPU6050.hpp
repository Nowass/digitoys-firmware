#pragma once
#include "I2C.hpp"
#include "IIMUSensor.hpp"
#include <stdint.h>

/// Driver for the InvenSense MPU6050 IMU over I2C
class MPU6050 : public IIMUSensor
{
public:
    /// Configuration parameters for MPU6050
    struct Config
    {
        enum AccelRange : uint8_t
        {
            ACCEL_RANGE_2G = 0,
            ACCEL_RANGE_4G,
            ACCEL_RANGE_8G,
            ACCEL_RANGE_16G
        };

        enum GyroRange : uint8_t
        {
            GYRO_RANGE_250 = 0,
            GYRO_RANGE_500,
            GYRO_RANGE_1000,
            GYRO_RANGE_2000
        };

        enum DLPF : uint8_t
        {
            DLPF_260HZ = 0,
            DLPF_184HZ,
            DLPF_94HZ,
            DLPF_44HZ,
            DLPF_21HZ,
            DLPF_10HZ,
            DLPF_5HZ
        };

        AccelRange accelRange = ACCEL_RANGE_8G; ///< Full-scale accel range
        GyroRange gyroRange = GYRO_RANGE_500;   ///< Full-scale gyro range
        DLPF dlpf = DLPF_44HZ;                  ///< Digital low-pass filter
        uint8_t sampleRateDiv = 19;             ///< Sample rate divider (50Hz with 1kHz internal rate)
        float filterAlpha = 0.2f;               ///< IIR filter α for smoothing
        uint8_t i2c_address = 0x68;             ///< Device I2C address (0x68 or 0x69)
    };

    MPU6050(I2C &bus, const Config &cfg);

    bool init() override;
    bool selfTest() override;
    bool dataReady() override;
    float getAccelX() override;
    float getAccelY() override;
    float getAccelZ() override;
    float getGyroX() override;
    float getGyroY() override;
    float getGyroZ() override;
    float getTemperature() override;
    float getAccelLongitudinal() override;

    /// Read all sensor data at once for efficiency
    struct SensorData
    {
        float accelX, accelY, accelZ; ///< Acceleration in m/s²
        float gyroX, gyroY, gyroZ;    ///< Angular velocity in deg/s
        float temperature;            ///< Temperature in °C
    };

    bool readAll(SensorData &data);

    /// Read raw ADC values for hardware diagnostics
    bool readRawADC(int16_t &accelX, int16_t &accelY, int16_t &accelZ, int16_t &temp);

    ~MPU6050() override = default;

private:
    I2C &_bus;
    Config _cfg;

    // Filtered values
    float _lastFilteredAccelX = 0.0f;
    float _lastFilteredAccelY = 0.0f;
    float _lastFilteredAccelZ = 0.0f;
    float _lastFilteredGyroX = 0.0f;
    float _lastFilteredGyroY = 0.0f;
    float _lastFilteredGyroZ = 0.0f;

    // Scale factors
    float _accelScale = 0.0f; // m/s² per LSB
    float _gyroScale = 0.0f;  // deg/s per LSB

    // Register access
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);
    bool readRegisters(uint8_t startReg, uint8_t *buf, size_t len);

    // Utility functions
    void applyIIRFilter(float &state, float raw);
    void calculateScaleFactors();
    bool resetDevice();
    bool configureSensor();

    // MPU6050 Register addresses
    static constexpr uint8_t REG_SELF_TEST_X = 0x0D;
    static constexpr uint8_t REG_SELF_TEST_Y = 0x0E;
    static constexpr uint8_t REG_SELF_TEST_Z = 0x0F;
    static constexpr uint8_t REG_SELF_TEST_A = 0x10;
    static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
    static constexpr uint8_t REG_CONFIG = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_FIFO_EN = 0x23;
    static constexpr uint8_t REG_I2C_MST_CTRL = 0x24;
    static constexpr uint8_t REG_INT_PIN_CFG = 0x37;
    static constexpr uint8_t REG_INT_ENABLE = 0x38;
    static constexpr uint8_t REG_INT_STATUS = 0x3A;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_ACCEL_XOUT_L = 0x3C;
    static constexpr uint8_t REG_ACCEL_YOUT_H = 0x3D;
    static constexpr uint8_t REG_ACCEL_YOUT_L = 0x3E;
    static constexpr uint8_t REG_ACCEL_ZOUT_H = 0x3F;
    static constexpr uint8_t REG_ACCEL_ZOUT_L = 0x40;
    static constexpr uint8_t REG_TEMP_OUT_H = 0x41;
    static constexpr uint8_t REG_TEMP_OUT_L = 0x42;
    static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
    static constexpr uint8_t REG_GYRO_XOUT_L = 0x44;
    static constexpr uint8_t REG_GYRO_YOUT_H = 0x45;
    static constexpr uint8_t REG_GYRO_YOUT_L = 0x46;
    static constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
    static constexpr uint8_t REG_GYRO_ZOUT_L = 0x48;
    static constexpr uint8_t REG_USER_CTRL = 0x6A;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
    static constexpr uint8_t REG_FIFO_COUNTH = 0x72;
    static constexpr uint8_t REG_FIFO_COUNTL = 0x73;
    static constexpr uint8_t REG_FIFO_R_W = 0x74;
    static constexpr uint8_t REG_WHO_AM_I = 0x75;

    // Expected WHO_AM_I values
    static constexpr uint8_t WHO_AM_I_VALUE_MPU6050 = 0x68;
    static constexpr uint8_t WHO_AM_I_VALUE_MPU6000 = 0x70;
};
