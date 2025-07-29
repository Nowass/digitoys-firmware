# MPU6050 Motion Tracking Extension

## 🚗 **Vehicle Speed and Braking Detection**

The MPU6050 driver has been enhanced with advanced motion tracking capabilities specifically designed for automotive applications. It now provides real-time speed estimation and automatic braking detection.

## 🎯 **Key Features**

### **Speed Detection**
- **Real-time Speed Estimation**: Integrates acceleration over time to calculate current speed
- **Units**: Provides speed in both m/s and km/h
- **Noise Filtering**: Eliminates low-speed drift with configurable thresholds
- **Direction**: Assumes forward motion (prevents negative speeds)

### **Braking Detection**
- **Automatic Detection**: Monitors for deceleration events above threshold
- **Braking Threshold**: Configurable (default: 2.0 m/s² deceleration)
- **Braking Metrics**: Tracks deceleration magnitude, start speed, and speed delta
- **Event Tracking**: Records braking start time and duration

### **Data Smoothing**
- **Acceleration Smoothing**: Uses exponential moving average (α = 0.8)
- **Noise Reduction**: Filters out sensor noise and vibrations
- **Drift Prevention**: Automatic speed reset when vehicle is stationary

## 📊 **Available Data**

### **Speed Information**
```cpp
float currentSpeed = mpu6050.getCurrentSpeed();           // m/s
float currentSpeedKmh = currentSpeed * 3.6f;            // km/h
float currentAccel = mpu6050.getCurrentAcceleration();   // m/s²
```

### **Braking Information**
```cpp
bool isBraking = mpu6050.isBraking();                    // true during braking
float brakingDecel = mpu6050.getBrakingDeceleration();   // m/s² (positive)
float startSpeed = mpu6050.getBrakingStartSpeed();       // m/s when braking started
float speedDelta = mpu6050.getBrakingSpeedDelta();       // m/s speed lost during braking
```

### **Utility Functions**
```cpp
mpu6050.resetSpeed();                                    // Reset speed to 0 (when stopped)
```

## 🔧 **Configuration Parameters**

### **Tunable Constants**
```cpp
static constexpr float BRAKING_THRESHOLD = 2.0f;        // m/s² deceleration threshold
static constexpr float SPEED_NOISE_THRESHOLD = 0.1f;    // m/s minimum speed change
static constexpr float ACCEL_ALPHA = 0.8f;              // Acceleration smoothing (0-1)
```

### **Axis Configuration**
- **Forward Direction**: X-axis (adjust based on sensor mounting)
- **Sensor Mounting**: Ensure X-axis points forward in vehicle
- **Calibration**: Must be performed with vehicle stationary

## 📈 **Sample Output**

```
I (xxx) MPU6050_EXAMPLE: Speed: 15.30 m/s (55.1 km/h), Accel: 0.25 m/s²
I (xxx) MPU6050_EXAMPLE: Speed: 12.80 m/s (46.1 km/h), Accel: -3.20 m/s²
I (xxx) MPU6050_EXAMPLE: BRAKING: Decel=3.20 m/s², Start=15.30 m/s, Delta=2.50 m/s
```

## 🚨 **Important Considerations**

### **Calibration Requirements**
1. **Stationary Calibration**: Must be performed with vehicle completely stopped
2. **Sensor Orientation**: X-axis must point in forward direction
3. **Level Surface**: Calibrate on level ground for best accuracy

### **Accuracy Factors**
- **Integration Drift**: Speed estimates may drift over time
- **Vibration Filtering**: Smoothing reduces high-frequency noise
- **Slope Compensation**: Does not compensate for road incline
- **Turning Effects**: May show lateral acceleration during turns

### **Automatic Drift Correction**
The driver includes automatic speed reset when:
- Speed < 0.1 m/s for 10 seconds
- Acceleration < 0.5 m/s² for 10 seconds
- Prevents long-term drift accumulation

## 🔄 **Integration with Vehicle Systems**

### **Braking System Integration**
```cpp
// Example: Integrate with braking control
if (mpu6050.isBraking()) {
    float brakingForce = mpu6050.getBrakingDeceleration();
    float speedLoss = mpu6050.getBrakingSpeedDelta();
    
    // Log braking event
    ESP_LOGI(TAG, "Braking detected: %.1f m/s² deceleration, %.1f m/s speed loss", 
             brakingForce, speedLoss);
    
    // Could trigger brake assistance or logging
}
```

### **Speed Monitoring**
```cpp
// Example: Speed-based vehicle control
float currentSpeedKmh = mpu6050.getCurrentSpeed() * 3.6f;
if (currentSpeedKmh > MAX_SPEED_KMH) {
    // Trigger speed warning or control
    ESP_LOGW(TAG, "Speed limit exceeded: %.1f km/h", currentSpeedKmh);
}
```

## ⚙️ **Advanced Configuration**

### **Mounting Orientation**
If sensor is mounted differently, modify the acceleration source:
```cpp
// In updateMotionTracking() function
float rawAccel = data.accelY;  // If Y-axis is forward
// or
float rawAccel = data.accelZ;  // If Z-axis is forward
```

### **Sensitivity Tuning**
```cpp
// More sensitive braking detection
static constexpr float BRAKING_THRESHOLD = 1.5f;  // Lower threshold

// Less sensitive to noise
static constexpr float SPEED_NOISE_THRESHOLD = 0.2f;  // Higher threshold
```

## 🎯 **Use Cases**

1. **Autonomous Braking**: Monitor braking events and effectiveness
2. **Speed Logging**: Track vehicle speed over time
3. **Driving Analytics**: Analyze acceleration and braking patterns
4. **Safety Systems**: Detect emergency braking situations
5. **Performance Monitoring**: Track acceleration and deceleration capabilities

The enhanced MPU6050 driver now provides comprehensive motion tracking suitable for automotive applications, with automatic calibration and intelligent filtering to ensure reliable operation in real-world conditions.
