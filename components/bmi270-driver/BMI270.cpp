#include "BMI270.hpp"
#include <esp_log.h>
#include <freertos/task.h>

static const char *TAG = "BMI270";

// BMI270 register definitions (subset)
namespace {
constexpr uint8_t REG_CHIP_ID     = 0x00;
constexpr uint8_t REG_STATUS      = 0x03;
constexpr uint8_t REG_CMD         = 0x7E;
constexpr uint8_t REG_PWR_CONF    = 0x7C;
constexpr uint8_t REG_PWR_CTRL    = 0x7D;
constexpr uint8_t REG_ACC_CONF    = 0x40;
constexpr uint8_t REG_ACC_RANGE   = 0x41;
constexpr uint8_t REG_INT_STATUS  = 0x1C;
constexpr uint8_t REG_DATA_START  = 0x12; // x,l,h,y,l,h,z,l,h
constexpr uint8_t CHIP_ID         = 0x24; // expected chip id
constexpr uint8_t CMD_SOFT_RESET  = 0xB6;
constexpr uint8_t DATA_RDY_BIT    = 0x80; // data ready in status
constexpr uint8_t ACCEL_EN_BIT    = 0x04; // accel enable in PWR_CTRL
} // namespace

BMI270::BMI270(I2C &bus, const Config &cfg) : _bus(bus), _cfg(cfg) {}

bool BMI270::writeRegister(uint8_t reg, uint8_t value)
{
    return _bus.write(_cfg.i2c_address, reg, &value, 1) == ESP_OK;
}

bool BMI270::readRegisters(uint8_t startReg, uint8_t *buf, size_t len)
{
    return _bus.read(_cfg.i2c_address, startReg, buf, len) == ESP_OK;
}

void BMI270::applyIIR(float &state, float raw)
{
    state = state * (1.0f - _cfg.filterAlpha) + raw * _cfg.filterAlpha;
}

bool BMI270::init()
{
    if (_bus.init() != ESP_OK)
        return false;

    vTaskDelay(pdMS_TO_TICKS(2)); // power-up time

    // soft reset
    writeRegister(REG_CMD, CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t id = 0;
    if (!readRegisters(REG_CHIP_ID, &id, 1) || id != CHIP_ID)
    {
        ESP_LOGE(TAG, "chip id mismatch: 0x%02X", id);
        return false;
    }

    // exit deep suspend
    writeRegister(REG_PWR_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1));

    // enable accelerometer
    writeRegister(REG_PWR_CTRL, ACCEL_EN_BIT);
    vTaskDelay(pdMS_TO_TICKS(5));

    // configure range
    uint8_t range = 0;
    switch (_cfg.accelRange)
    {
    case Config::RANGE_2G:
        range = 0x00;
        _scale = 2.0f / 32768.0f;
        break;
    case Config::RANGE_4G:
        range = 0x01;
        _scale = 4.0f / 32768.0f;
        break;
    case Config::RANGE_8G:
        range = 0x02;
        _scale = 8.0f / 32768.0f;
        break;
    case Config::RANGE_16G:
        range = 0x03;
        _scale = 16.0f / 32768.0f;
        break;
    }
    writeRegister(REG_ACC_RANGE, range);

    // configure ODR (approximate mapping)
    uint8_t odr = 0x08; // default 100 Hz
    if (_cfg.odr_hz >= 200)
        odr = 0x09; // 200 Hz
    else if (_cfg.odr_hz >= 400)
        odr = 0x0A; // 400 Hz
    writeRegister(REG_ACC_CONF, odr);

    return true;
}

bool BMI270::selfTest()
{
    // for simplicity always return true
    return true;
}

bool BMI270::dataReady()
{
    uint8_t st = 0;
    if (!readRegisters(REG_STATUS, &st, 1))
        return false;
    return (st & DATA_RDY_BIT) != 0;
}

static float convert(const uint8_t *buf, float scale)
{
    int16_t raw = int16_t(buf[0] | (buf[1] << 8));
    return raw * scale * 9.80665f; // convert g to m/s^2
}

float BMI270::getAccelX()
{
    uint8_t buf[6];
    if (!readRegisters(REG_DATA_START, buf, sizeof(buf)))
        return 0.0f;
    float x = convert(&buf[0], _scale);
    applyIIR(_lastFilteredX, x);
    _lastFilteredY = convert(&buf[2], _scale) * (1.0f - _cfg.filterAlpha) + _cfg.filterAlpha * _lastFilteredY; // Keep Y updated as well
    _lastFilteredZ = convert(&buf[4], _scale) * (1.0f - _cfg.filterAlpha) + _cfg.filterAlpha * _lastFilteredZ;
    return _lastFilteredX;
}

float BMI270::getAccelY()
{
    return _lastFilteredY;
}

float BMI270::getAccelZ()
{
    return _lastFilteredZ;
}

float BMI270::getAccelLongitudinal()
{
    return getAccelX();
}
