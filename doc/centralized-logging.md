# 📋 DigiToys Centralized Logging System

## Overview

The DigiToys firmware features a **production-ready centralized logging system** that provides unified logging management across all components. This system builds upon ESP-IDF's logging infrastructure while adding component registration, simplified API, and centralized configuration.

## ✅ Current Implementation Status

**🎯 Status**: **PRODUCTION READY** - Fully implemented and deployed across all components

**🏗️ System Architecture**:
- **Centralized Logger**: Singleton managing all component logging
- **Simplified API**: 2-parameter macros (component name only)
- **Automatic Registration**: Components register once during initialization
- **Tag Management**: Automatic tag lookup, no redundancy

---

## 🚀 Key Features

### 1. **Simplified Logging API**

**Before** (Legacy - 3 parameters):
```cpp
DIGITOYS_LOGI("PwmDriver", "PWM", "Component started successfully");
```

**After** (Current - 2 parameters):
```cpp
// Register once during component initialization
DIGITOYS_REGISTER_COMPONENT("PwmDriver", "PWM");

// Use everywhere - much cleaner!
DIGITOYS_LOGI("PwmDriver", "Component started successfully");
DIGITOYS_LOGW("PwmDriver", "Component not running");
DIGITOYS_LOGE("PwmDriver", "Failed with error: %d", error_code);
```

### 2. **Automatic Component Registration**

Each component registers itself once with a human-readable name and logging tag:

```cpp
// In component constructor or initialization
DIGITOYS_REGISTER_COMPONENT("LiDAR", "LIDAR");
DIGITOYS_REGISTER_COMPONENT("PwmDriver", "PWM");
DIGITOYS_REGISTER_COMPONENT("ControlTask", "CONTROL");
DIGITOYS_REGISTER_COMPONENT("Monitor", "MONITOR");
```

### 3. **Per-Component Log Level Control**

```cpp
auto& logger = digitoys::core::Logger::getInstance();

// Set specific log levels for different components
logger.setComponentLogLevel("LiDAR", ESP_LOG_DEBUG);
logger.setComponentLogLevel("ControlTask", ESP_LOG_VERBOSE);
logger.setComponentLogLevel("Monitor", ESP_LOG_WARN);

// Enable/disable entire components
logger.setComponentEnabled("Monitor", false);

// Global debug mode for all components
logger.setDebugMode(true);
```

### 4. **Production Component Coverage**

**✅ All Major Components Migrated**:
- **PwmDriver** (adas-pwm-driver): PWM signal processing
- **LiDAR** (lidar-driver): Distance sensor management  
- **ControlTask** (control-task): Main control logic
- **RCProcessor** (control-task): RC input processing
- **Monitor** (monitor): Web interface and telemetry
- **SystemMonitor** (monitor): System health monitoring

**✅ Supporting Infrastructure**:
- **ComponentBase**: Base class for all components
- **ConfigValidator**: Configuration validation
- **ConfigFactory**: Configuration creation
- **All Config Classes**: PWM and LiDAR configuration

---

## 📝 Usage Guide

### 1. **Component Registration** (Required Once)

Every component must register itself to use the centralized logging:

```cpp
class MyComponent : public digitoys::core::ComponentBase {
public:
    MyComponent() : ComponentBase("MyComponent") {
        // Register with centralized logging system
        DIGITOYS_REGISTER_COMPONENT("MyComponent", "MY_TAG");
    }
};
```

### 2. **Basic Logging** (Standard Usage)

```cpp
#include <Logger.hpp>

void myFunction() {
    // Simple logging - component name only
    DIGITOYS_LOGI("MyComponent", "Operation started");
    DIGITOYS_LOGW("MyComponent", "Warning condition detected");
    DIGITOYS_LOGE("MyComponent", "Error occurred: %s", error_message);
    
    // With formatting
    DIGITOYS_LOGI("MyComponent", "Processing %d items at %.2f rate", 
                  item_count, processing_rate);
}
```

### 3. **Debug Logging** (Development)

```cpp
// Debug messages only appear when debug level is enabled
DIGITOYS_LOGD("MyComponent", "Debug info: state=%d, value=%.3f", state, value);
DIGITOYS_LOGV("MyComponent", "Verbose trace: entering function %s", __func__);
```

### 4. **Convenience Macros** (For Common Components)

Pre-defined shortcuts for frequently used components:

```cpp
// Shorthand for common components
LOG_LIDAR(I, "Distance reading: %.2f meters", distance);
LOG_PWM(W, "PWM signal out of range: %.3f", duty_cycle);
LOG_CONTROL(E, "Emergency brake triggered");
LOG_MONITOR(I, "HTTP server started on port %d", port);
LOG_SYSTEM(D, "Free heap: %u bytes", heap_size);
LOG_MAIN(I, "System initialization complete");
```

---

## 🔧 Runtime Configuration

### Component-Level Control

```cpp
auto& logger = digitoys::core::Logger::getInstance();

// Individual component log levels
logger.setComponentLogLevel("LiDAR", ESP_LOG_DEBUG);      // Show all LiDAR logs
logger.setComponentLogLevel("Monitor", ESP_LOG_ERROR);     // Only errors from Monitor
logger.setComponentLogLevel("PwmDriver", ESP_LOG_INFO);    // Normal PWM logging

// Disable specific components
logger.setComponentEnabled("SystemMonitor", false);       // Silence system stats

// Query component status
if (logger.isLoggingEnabled("ControlTask", ESP_LOG_DEBUG)) {
    // Expensive debug operation only when needed
    performDetailedDiagnostics();
}
```

### Global Settings

```cpp
// Global debug mode (affects all components)
logger.setDebugMode(true);                    // Enable debug for all
logger.setGlobalLogLevel(ESP_LOG_WARN);       // Global minimum level

// Debug specific components only
logger.enableDebugFor({"LiDAR", "ControlTask"});
```

### Component Diagnostics

```cpp
// Get component information
const auto* info = logger.getComponentInfo("LiDAR");
if (info) {
    DIGITOYS_LOGI("Main", "Component: %s, Tag: %s, Messages: %u, Level: %d",
                  info->name.c_str(), info->tag.c_str(), 
                  info->message_count, info->level);
}

// List all registered components
for (const auto& [name, info] : logger.getAllComponents()) {
    DIGITOYS_LOGI("Main", "Component: %s -> Tag: %s (%u messages)",
                  name.c_str(), info.tag.c_str(), info.message_count);
}
```

---

## � System Architecture

### Registration Flow
1. **Component Init**: `DIGITOYS_REGISTER_COMPONENT("Name", "TAG")`
2. **Logger Storage**: Component info stored in singleton
3. **Runtime Lookup**: Logging macros find tag by component name
4. **ESP-IDF Integration**: Forwards to ESP_LOG with proper tag

### Tag Assignment
- **LiDAR Component** → `LIDAR` tag
- **PwmDriver Component** → `PWM` tag  
- **ControlTask Component** → `CONTROL` tag
- **Monitor Component** → `MONITOR` tag
- **SystemMonitor Component** → `MONITOR` tag (shared)

### Memory Efficiency
- **No per-component storage**: Uses singleton pattern
- **Efficient lookup**: std::unordered_map for O(1) component resolution
- **ESP-IDF integration**: Leverages existing ESP_LOG infrastructure

---

## 🧪 Testing & Examples

### Demo Application

See `main_centralized_logging_demo.cpp` for complete examples:

```cpp
// Component registration
DIGITOYS_REGISTER_COMPONENT("TestComponent", "TEST");

// Basic logging
DIGITOYS_LOGI("TestComponent", "System started");

// Dynamic level changes
logger.setComponentLogLevel("TestComponent", ESP_LOG_DEBUG);
DIGITOYS_LOGD("TestComponent", "Debug message now visible");

// Global debug mode
logger.setDebugMode(true);
DIGITOYS_LOGD("TestComponent", "Global debug enabled");
```

### Build Status

✅ **Compilation**: Clean build, no warnings  
✅ **Runtime**: Tag assignment bug fixed  
✅ **Production**: All 6 major components migrated  
✅ **API**: Simplified 2-parameter interface deployed  

---

## 🎯 Benefits Achieved

### 1. **Developer Experience**
- **50% fewer parameters**: No more redundant tag specification
- **Error prevention**: Can't mismatch component names and tags
- **IDE friendly**: Better autocomplete and intellisense
- **Consistent naming**: Component names match their logging identity

### 2. **Maintenance**
- **Single source of truth**: Tag changes in one place only
- **Easy debugging**: Per-component log level control
- **Runtime flexibility**: Change logging without rebuilding
- **Clear organization**: Hierarchical component structure

### 3. **Production Ready**
- **Performance optimized**: Efficient tag lookup
- **Memory efficient**: No per-component overhead
- **ESP-IDF compatible**: Seamless integration with existing tools
- **Robust**: Comprehensive error handling and validation

---

## 🚀 Advanced Features (Future)

The current system provides a solid foundation for advanced logging features:

### Potential Enhancements
- **Remote Logging**: WiFi/HTTP log streaming
- **Log Filtering**: Advanced pattern-based filtering  
- **Structured Logging**: JSON output for machine processing
- **Performance Monitoring**: Log frequency analytics
- **Web Dashboard**: Real-time log viewing interface

### Integration Points
- **Monitor Component**: Ready for log viewing UI
- **Configuration System**: Runtime log level persistence
- **Telemetry**: Log statistics in system monitoring

---

This centralized logging system provides **production-grade logging infrastructure** with a **clean, simplified API** that makes debugging and maintenance significantly easier while maintaining high performance and ESP-IDF compatibility.
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
