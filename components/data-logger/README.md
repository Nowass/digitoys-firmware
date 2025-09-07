# Data Logger Component

## Overview

The `data-logger` component provides a generic framework for collecting and managing development data from various sources across the digitoys system. It follows the digitoys-core framework patterns and is designed specifically for development builds to avoid impacting production performance.

## Features

- ✅ **Component Lifecycle**: Full ComponentBase implementation with proper state management
- ✅ **Configurable**: Enable/disable via configuration, memory limits, auto-flush intervals
- ✅ **Memory Management**: Tracks memory usage and enforces limits
- ✅ **Auto-flush**: Periodic data flushing with configurable intervals
- ✅ **IDataSource Interface**: Generic interface for any component to provide data
- ✅ **Data Source Registry**: Register/unregister sources dynamically
- ✅ **Test Data Source**: Synthetic data generation for validation and testing
- ✅ **Demo Framework**: Complete example showing integration patterns
- 🔄 **Extensible**: Ready for specialized analyzers

## Architecture

```
DataLogger (ComponentBase)
├── Configuration Management
├── Memory Tracking
├── Auto-flush Timer
└── [Future] Data Source Registry
```

## Configuration

```cpp
struct DataLoggerConfig {
    bool enabled = false;                    // Enable/disable logging
    uint32_t max_entries = 1000;           // Maximum number of entries  
    uint32_t flush_interval_ms = 5000;     // Auto-flush interval
    size_t max_memory_kb = 64;             // Maximum memory usage in KB
};
```

## Usage

### Basic DataLogger Usage

```cpp
#include "DataLogger.hpp"

// Create logger with custom config
DataLoggerConfig config = {
    .enabled = true,
    .max_entries = 500,
    .flush_interval_ms = 3000,
    .max_memory_kb = 32
};

DataLogger logger(config);

// Standard component lifecycle
logger.initialize();
logger.start();

// Use logger...
// logger.isEnabled()
// logger.getMemoryUsage()
// logger.getEntryCount()

// Cleanup
logger.stop();
logger.shutdown();
```

### Test Data Source Usage

```cpp
#include "TestDataSource.hpp"
#include "DataLogger.hpp"

// Create test data source
auto testSource = std::make_shared<TestDataSource>("TestSensor", 250, true);
testSource->setDataPattern(TestDataSource::DataPattern::SINUSOIDAL);
testSource->setSimulateFailures(false);

// Register with logger
logger.registerDataSource(testSource);

// Data will be collected automatically
// Check statistics: testSource->getDataPointsGenerated()
```

### Complete Demo Example

```cpp
#include "DataLoggerDemo.hpp"

// Initialize and start demo
DataLoggerDemo::initialize();
DataLoggerDemo::start();

// Monitor progress
DataLoggerDemo::printStatus();

// Stop demo
DataLoggerDemo::stop();
```

## Dependencies

- `digitoys-core`: ComponentBase interface and common utilities
- `freertos`: Timer management
- `esp_timer`: High-resolution timers for auto-flush

## Build Integration

The component is automatically built when included in the project dependencies. No additional configuration required.

## Development Status

**Phase 1: Core Framework** ✅ **COMPLETED**
- Basic component structure
- Configuration system
- Memory tracking
- Auto-flush mechanism

**Phase 2: Data Source Interface** ✅ **COMPLETED**
- IDataSource interface
- Source registration
- Generic data collection

**Phase 3: Test & Validation** ✅ **COMPLETED**
- TestDataSource implementation
- Demo framework
- Synthetic data patterns

**Phase 4: Specialized Analyzers** 📋 **NEXT**
- PhysicsAnalyzer for control system data
- Custom analyzer framework
