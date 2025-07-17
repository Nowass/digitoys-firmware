# 🟢 Component: bmi270-driver

This component provides a minimal driver for the Bosch **BMI270** 3‑axis accelerometer. The sensor is accessed over the I²C bus and exposes a small C++ API that fits into the existing project style.

## Basic blocks

### `I2C`
- Thin wrapper around the ESP‑IDF I2C master driver
- Handles bus configuration and simple register read/write helpers

### `BMI270`
- Implements `IAccelSensor`
- Configures output data rate and measurement range
- Uses an IIR filter to smooth raw readings
- Polls the sensor's data‑ready status bit

Although the API exposes `dataReady()`, the driver can be extended to trigger an interrupt callback when the BMI270's DRDY pin is wired to a GPIO. Using the sensor's FIFO together with an IRQ minimises CPU load because samples are batched and only processed when ready.

## Usage example

```cpp
I2C::Config busCfg{ I2C_NUM_0, GPIO_NUM_4, GPIO_NUM_5, 400000 };
static I2C bus(busCfg);
BMI270::Config accelCfg;
BMI270 accel(bus, accelCfg);

ESP_ERROR_CHECK(accel.init());
if (accel.selfTest()) {
    if (accel.dataReady()) {
        float ax = accel.getAccelLongitudinal();
        // ...
    }
}
```

The component registers with CMake like the others and depends only on the ESP‑IDF `driver` component.
