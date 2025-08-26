# 📋 DigiToys Centralized Logging System

## Overview

The DigiToys firmware now includes a centralized logging system that provides unified logging management across all components. This system builds upon ESP-IDF's logging infrastructure while adding component registration, consistent formatting, and centralized configuration.

## ✅ What We've Accomplished

### Phase 4: Centralized Logging Implementation

**🎯 Current State**: Successfully implemented centralized logging infrastructure with backward compatibility.

**🔧 Infrastructure Components**:
1. **Logger Class** (`digitoys-core/Logger.hpp`): Central logging coordinator
2. **Component Registration**: Automatic TAG management with "DT_" prefix
3. **Unified Constants**: Centralized TAG and component name definitions
4. **Backward Compatibility**: Legacy macros for existing code

---

## 🚀 New Centralized Logging Features

### 1. **Automatic Component Registration**

```cpp
// Components are automatically registered on first use
LOG_LIDAR(I, "LiDAR sensor initialized successfully");
// Creates component "LiDAR" with TAG "DT_LIDAR"
```

### 2. **Consistent TAG Management**

All components now use standardized TAGs:
- **Main**: `DT_MAIN`
- **LiDAR**: `DT_LIDAR` 
- **PWM**: `DT_PWM`
- **Control**: `DT_CONTROL`
- **Monitor**: `DT_MONITOR`
- **System**: `DT_SYSTEM`

### 3. **Component-Aware Logging**

```cpp
// New centralized macros with automatic registration
DIGITOYS_LOGI("ComponentName", "TAG", "Message with %s", "formatting");

// Convenience macros for common components
LOG_LIDAR(I, "Distance reading: %.2f meters", distance);
LOG_PWM(W, "PWM signal out of range: %.3f", duty_cycle);
LOG_CONTROL(E, "Emergency brake triggered at distance %.2f", distance);
```

### 4. **Per-Component Log Level Control**

```cpp
auto& logger = digitoys::core::Logger::getInstance();

// Set specific log levels for different components
logger.setComponentLogLevel("LiDAR", ESP_LOG_DEBUG);
logger.setComponentLogLevel("Control", ESP_LOG_VERBOSE);
logger.setComponentLogLevel("Monitor", ESP_LOG_WARN);
```

### 5. **Component Diagnostics**

```cpp
// Get logging statistics for any component
const auto* info = logger.getComponentInfo("LiDAR");
if (info) {
    ESP_LOGI("MAIN", "LiDAR logged %u messages, level: %d", 
             info->message_count, info->level);
}

// List all registered components
for (const auto& [name, info] : logger.getAllComponents()) {
    ESP_LOGI("MAIN", "Component: %s, TAG: %s, Messages: %u",
             name.c_str(), info.tag.c_str(), info.message_count);
}
```

---

## 📝 Migration Guide

### Current Implementation Status

**✅ Infrastructure Ready**: Centralized logging system implemented and tested
**🔄 Migration Phase**: Components still using legacy macros for stability
**📋 Next Steps**: Gradual migration to new logging macros

### 1. **Immediate Usage** (New Code)

For new code, use the centralized logging macros:

```cpp
#include "Logger.hpp"

void myNewFunction() {
    // Use convenience macros
    LOG_LIDAR(I, "Starting new LiDAR operation");
    LOG_PWM(W, "PWM value %.3f exceeds safe range", value);
    
    // Or use full macro with custom component
    DIGITOYS_LOGI("MyComponent", "MY_TAG", "Custom component message");
}
```

### 2. **Legacy Code Compatibility**

Existing code continues to work unchanged:

```cpp
// This still works - uses ESP-IDF logging directly
static const char* TAG = "MY_COMPONENT";
ESP_LOGI(TAG, "Traditional logging message");

// Legacy digitoys macros also work
DIGITOYS_LOGI_LEGACY(TAG, "Legacy macro usage");
```

### 3. **Gradual Migration Strategy**

**Phase 4.1** (Current): Infrastructure in place, legacy macros for stability
**Phase 4.2** (Next): Update one component at a time to new logging
**Phase 4.3** (Future): Remove legacy macros, full centralized logging

---

## 🔧 Configuration

### Default Configuration

Components are automatically registered with these defaults:
- **Log Level**: `ESP_LOG_INFO`
- **TAG Prefix**: `"DT_"`
- **Enabled**: `true`

### Runtime Configuration

```cpp
auto& logger = digitoys::core::Logger::getInstance();

// Change log levels
logger.setComponentLogLevel("LiDAR", ESP_LOG_DEBUG);

// Enable/disable component logging
logger.setComponentEnabled("Monitor", false);

// Check if logging is enabled for specific level
if (logger.isLoggingEnabled("Control", ESP_LOG_DEBUG)) {
    // Expensive debug operation only if debug logging enabled
    performDetailedAnalysis();
}
```

---

## 📊 Benefits

### 1. **Centralized Management**
- Single point for log level configuration
- Consistent TAG naming across components
- Easy component enable/disable

### 2. **Enhanced Debugging**
- Function name automatically included: `[functionName] message`
- Component lifecycle tracking
- Message count statistics per component

### 3. **Better Organization**
- Standardized log format across all components
- Clear component identification with DT_ prefix
- Hierarchical log level control

### 4. **Development Efficiency**
- Automatic component registration
- No manual TAG management required
- Easy component-specific debugging

---

## 🧪 Testing & Validation

### Build Status
✅ **Compilation**: All components compile successfully with new infrastructure
✅ **Backward Compatibility**: Existing logging continues to work
✅ **Runtime**: Logger singleton properly manages component registration

### Demo Application

See `main_centralized_logging_demo.cpp` for complete usage examples including:
- Component registration demonstration
- Mixed logging approach (legacy + centralized)
- Runtime component information display

---

## 🚀 Next Steps

### Immediate (Phase 4.1 Complete)
- [x] Centralized Logger infrastructure
- [x] Component registration system
- [x] Unified constants and TAG management
- [x] Backward compatibility maintenance

### Phase 4.2: Component Migration
- [ ] Update LiDAR component to use centralized logging
- [ ] Update PWM component to use centralized logging  
- [ ] Update Control Task to use centralized logging
- [ ] Update Monitor components to use centralized logging

### Phase 4.3: Full Integration
- [ ] Remove legacy macro dependencies
- [ ] Add advanced logging features (filtering, routing)
- [ ] Integration with Monitor component for log viewing
- [ ] Performance optimization for high-frequency logging

---

## 💡 Usage Examples

### Basic Component Logging
```cpp
// Simple component logging
LOG_LIDAR(I, "Sensor initialized");
LOG_PWM(W, "Signal weak");
LOG_CONTROL(E, "Emergency stop");
```

### Advanced Logging with Data
```cpp
// Structured logging with parameters
LOG_CONTROL(I, "Brake calculation: distance=%.2fm, speed=%.1f, brake_duty=%.3f",
           obstacle_distance, current_speed, brake_duty);

// Debug logging (only appears if debug level enabled)
LOG_LIDAR(D, "Raw sensor data: angle=%d, distance=%d, confidence=%d",
          point.angle, point.distance, point.confidence);
```

### Custom Component Registration
```cpp
// For specialized components not covered by convenience macros
DIGITOYS_LOGI("CustomSensor", "CUSTOM", "Specialized component message");
```

---

This centralized logging system provides a solid foundation for improved debugging, monitoring, and maintenance of the DigiToys firmware while maintaining full backward compatibility with existing code.
