#include "MPU6050.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>

static const char *TAG = "MPU6050";

MPU6050::MPU6050(I2C &bus, const Config &cfg) : _bus(bus), _cfg(cfg)
{
    calculateScaleFactors();
}

bool MPU6050::init()
{
    ESP_LOGI(TAG, "Initializing MPU6050...");

    // Reset device
    if (!resetDevice())
    {
        ESP_LOGE(TAG, "Failed to reset device");
        return false;
    }

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check WHO_AM_I register
    uint8_t whoAmI;
    if (!readRegister(REG_WHO_AM_I, whoAmI))
    {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return false;
    }

    if (whoAmI != WHO_AM_I_VALUE_MPU6050 && whoAmI != WHO_AM_I_VALUE_MPU6000)
    {
        ESP_LOGE(TAG, "Invalid WHO_AM_I value: 0x%02X, expected 0x%02X or 0x%02X",
                 whoAmI, WHO_AM_I_VALUE_MPU6050, WHO_AM_I_VALUE_MPU6000);
        return false;
    }

    ESP_LOGI(TAG, "WHO_AM_I check passed: 0x%02X", whoAmI);

    // Configure sensor
    if (!configureSensor())
    {
        ESP_LOGE(TAG, "Failed to configure sensor");
        return false;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return true;
}

bool MPU6050::selfTest()
{
    ESP_LOGI(TAG, "Performing self-test...");

    // Store original configuration
    uint8_t originalGyroConfig, originalAccelConfig;
    if (!readRegister(REG_GYRO_CONFIG, originalGyroConfig) ||
        !readRegister(REG_ACCEL_CONFIG, originalAccelConfig))
    {
        ESP_LOGE(TAG, "Failed to read original configuration");
        return false;
    }

    // Enable self-test for gyro and accelerometer
    if (!writeRegister(REG_GYRO_CONFIG, 0xE0) || // Enable self-test, ±250°/s
        !writeRegister(REG_ACCEL_CONFIG, 0xE0))
    { // Enable self-test, ±2g
        ESP_LOGE(TAG, "Failed to enable self-test");
        return false;
    }

    // Wait for self-test to stabilize
    vTaskDelay(pdMS_TO_TICKS(250));

    // Read self-test response
    uint8_t selfTestData[6];
    if (!readRegisters(REG_ACCEL_XOUT_H, selfTestData, 6))
    {
        ESP_LOGE(TAG, "Failed to read self-test data");
        return false;
    }

    // Restore original configuration
    if (!writeRegister(REG_GYRO_CONFIG, originalGyroConfig) ||
        !writeRegister(REG_ACCEL_CONFIG, originalAccelConfig))
    {
        ESP_LOGE(TAG, "Failed to restore original configuration");
        return false;
    }

    // Wait for settings to take effect
    vTaskDelay(pdMS_TO_TICKS(50));

    // Basic validation: check if self-test responses are reasonable
    int16_t stResponse[3];
    stResponse[0] = (int16_t)((selfTestData[0] << 8) | selfTestData[1]);
    stResponse[1] = (int16_t)((selfTestData[2] << 8) | selfTestData[3]);
    stResponse[2] = (int16_t)((selfTestData[4] << 8) | selfTestData[5]);

    // Check if responses are within reasonable range (not zero, not saturated)
    for (int i = 0; i < 3; i++)
    {
        if (abs(stResponse[i]) < 100 || abs(stResponse[i]) > 30000)
        {
            ESP_LOGW(TAG, "Self-test response axis %d suspicious: %d", i, stResponse[i]);
        }
    }

    ESP_LOGI(TAG, "Self-test completed");
    return true;
}

bool MPU6050::dataReady()
{
    uint8_t status;
    if (!readRegister(REG_INT_STATUS, status))
    {
        return false;
    }
    return (status & 0x01) != 0; // Check DATA_RDY_INT bit
}

float MPU6050::getAccelX()
{
    uint8_t data[2];
    if (!readRegisters(REG_ACCEL_XOUT_H, data, 2))
    {
        return 0.0f;
    }
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    float accel = raw * _accelScale;
    applyIIRFilter(_lastFilteredAccelX, accel);
    return _lastFilteredAccelX;
}

float MPU6050::getAccelY()
{
    uint8_t data[2];
    if (!readRegisters(REG_ACCEL_YOUT_H, data, 2))
    {
        return 0.0f;
    }
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    float accel = raw * _accelScale;
    applyIIRFilter(_lastFilteredAccelY, accel);
    return _lastFilteredAccelY;
}

float MPU6050::getAccelZ()
{
    uint8_t data[2];
    if (!readRegisters(REG_ACCEL_ZOUT_H, data, 2))
    {
        return 0.0f;
    }
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    float accel = raw * _accelScale;
    applyIIRFilter(_lastFilteredAccelZ, accel);
    return _lastFilteredAccelZ;
}

float MPU6050::getGyroX()
{
    uint8_t data[2];
    if (!readRegisters(REG_GYRO_XOUT_H, data, 2))
    {
        return 0.0f;
    }
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    float gyro = raw * _gyroScale;
    applyIIRFilter(_lastFilteredGyroX, gyro);
    return _lastFilteredGyroX;
}

float MPU6050::getGyroY()
{
    uint8_t data[2];
    if (!readRegisters(REG_GYRO_YOUT_H, data, 2))
    {
        return 0.0f;
    }
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    float gyro = raw * _gyroScale;
    applyIIRFilter(_lastFilteredGyroY, gyro);
    return _lastFilteredGyroY;
}

float MPU6050::getGyroZ()
{
    uint8_t data[2];
    if (!readRegisters(REG_GYRO_ZOUT_H, data, 2))
    {
        return 0.0f;
    }
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    float gyro = raw * _gyroScale;
    applyIIRFilter(_lastFilteredGyroZ, gyro);
    return _lastFilteredGyroZ;
}

float MPU6050::getTemperature()
{
    uint8_t data[2];
    if (!readRegisters(REG_TEMP_OUT_H, data, 2))
    {
        return 0.0f;
    }
    int16_t raw = (int16_t)((data[0] << 8) | data[1]);
    // Temperature formula: Temperature in degrees C = (TEMP_OUT Register Value as a signed 16-bit value)/340 + 36.53
    return (raw / 340.0f) + 36.53f;
}

float MPU6050::getAccelLongitudinal()
{
    // Assuming X-axis is forward direction (adjust based on your mounting)
    return getAccelX();
}

bool MPU6050::readAll(SensorData &data)
{
    uint8_t buffer[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, buffer, 14))
    {
        return false;
    }

    // Parse accelerometer data
    int16_t accelX = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t accelY = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t accelZ = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Parse temperature data
    int16_t temp = (int16_t)((buffer[6] << 8) | buffer[7]);

    // Parse gyroscope data
    int16_t gyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);

    // Convert to physical units
    data.accelX = accelX * _accelScale;
    data.accelY = accelY * _accelScale;
    data.accelZ = accelZ * _accelScale;
    data.gyroX = gyroX * _gyroScale;
    data.gyroY = gyroY * _gyroScale;
    data.gyroZ = gyroZ * _gyroScale;
    data.temperature = (temp / 340.0f) + 36.53f;

    // Apply filtering
    applyIIRFilter(_lastFilteredAccelX, data.accelX);
    applyIIRFilter(_lastFilteredAccelY, data.accelY);
    applyIIRFilter(_lastFilteredAccelZ, data.accelZ);
    applyIIRFilter(_lastFilteredGyroX, data.gyroX);
    applyIIRFilter(_lastFilteredGyroY, data.gyroY);
    applyIIRFilter(_lastFilteredGyroZ, data.gyroZ);

    // Return filtered values
    data.accelX = _lastFilteredAccelX;
    data.accelY = _lastFilteredAccelY;
    data.accelZ = _lastFilteredAccelZ;
    data.gyroX = _lastFilteredGyroX;
    data.gyroY = _lastFilteredGyroY;
    data.gyroZ = _lastFilteredGyroZ;

    return true;
}

bool MPU6050::readRawADC(int16_t &accelX, int16_t &accelY, int16_t &accelZ, int16_t &temp)
{
    uint8_t buffer[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, buffer, 14))
    {
        return false;
    }

    // Parse raw ADC values (16-bit signed)
    accelX = (int16_t)((buffer[0] << 8) | buffer[1]);
    accelY = (int16_t)((buffer[2] << 8) | buffer[3]);
    accelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    temp = (int16_t)((buffer[6] << 8) | buffer[7]);

    return true;
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
    return _bus.writeByte(_cfg.i2c_address, reg, value) == ESP_OK;
}

bool MPU6050::readRegister(uint8_t reg, uint8_t &value)
{
    return _bus.readByte(_cfg.i2c_address, reg, &value) == ESP_OK;
}

bool MPU6050::readRegisters(uint8_t startReg, uint8_t *buf, size_t len)
{
    return _bus.read(_cfg.i2c_address, startReg, buf, len) == ESP_OK;
}

void MPU6050::applyIIRFilter(float &state, float raw)
{
    state = _cfg.filterAlpha * raw + (1.0f - _cfg.filterAlpha) * state;
}

void MPU6050::calculateScaleFactors()
{
    // Calculate accelerometer scale factor
    switch (_cfg.accelRange)
    {
    case Config::ACCEL_RANGE_2G:
        _accelScale = 9.81f / 16384.0f; // 9.81 m/s² per g, 16384 LSB/g
        break;
    case Config::ACCEL_RANGE_4G:
        _accelScale = 9.81f / 8192.0f;
        break;
    case Config::ACCEL_RANGE_8G:
        _accelScale = 9.81f / 4096.0f;
        break;
    case Config::ACCEL_RANGE_16G:
        _accelScale = 9.81f / 2048.0f;
        break;
    }

    // Calculate gyroscope scale factor
    switch (_cfg.gyroRange)
    {
    case Config::GYRO_RANGE_250:
        _gyroScale = 250.0f / 32768.0f;
        break;
    case Config::GYRO_RANGE_500:
        _gyroScale = 500.0f / 32768.0f;
        break;
    case Config::GYRO_RANGE_1000:
        _gyroScale = 1000.0f / 32768.0f;
        break;
    case Config::GYRO_RANGE_2000:
        _gyroScale = 2000.0f / 32768.0f;
        break;
    }
}

bool MPU6050::resetDevice()
{
    // Set DEVICE_RESET bit in PWR_MGMT_1 register
    if (!writeRegister(REG_PWR_MGMT_1, 0x80))
    {
        return false;
    }

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check if reset completed (bit should be cleared)
    uint8_t pwrMgmt1;
    if (!readRegister(REG_PWR_MGMT_1, pwrMgmt1))
    {
        return false;
    }

    return (pwrMgmt1 & 0x80) == 0;
}

bool MPU6050::configureSensor()
{
    // Wake up device (clear SLEEP bit)
    if (!writeRegister(REG_PWR_MGMT_1, 0x00))
    {
        ESP_LOGE(TAG, "Failed to wake up device");
        return false;
    }

    // Set sample rate divider
    if (!writeRegister(REG_SMPLRT_DIV, _cfg.sampleRateDiv))
    {
        ESP_LOGE(TAG, "Failed to set sample rate divider");
        return false;
    }

    // Configure DLPF
    if (!writeRegister(REG_CONFIG, _cfg.dlpf))
    {
        ESP_LOGE(TAG, "Failed to configure DLPF");
        return false;
    }

    // Configure gyroscope range
    if (!writeRegister(REG_GYRO_CONFIG, _cfg.gyroRange << 3))
    {
        ESP_LOGE(TAG, "Failed to configure gyroscope range");
        return false;
    }

    // Configure accelerometer range
    if (!writeRegister(REG_ACCEL_CONFIG, _cfg.accelRange << 3))
    {
        ESP_LOGE(TAG, "Failed to configure accelerometer range");
        return false;
    }

    // Enable data ready interrupt
    if (!writeRegister(REG_INT_ENABLE, 0x01))
    {
        ESP_LOGE(TAG, "Failed to enable data ready interrupt");
        return false;
    }

    ESP_LOGI(TAG, "Sensor configured: Accel ±%dg, Gyro ±%d°/s, DLPF %d, Sample rate div %d",
             2 << _cfg.accelRange,
             250 << _cfg.gyroRange,
             _cfg.dlpf,
             _cfg.sampleRateDiv);

    return true;
}
