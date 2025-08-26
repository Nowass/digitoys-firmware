# 🔵 Component: monitor

This component monitors system metrics such as heap usage, task runtime statistics, and CPU load. It is intended for diagnostics, debugging, and visualization via web dashboards or serial outputs.

## Basic blocks

### `SystemMonitor`
- Queries FreeRTOS APIs to extract:
  - Heap usage (free, min, total)
  - CPU usage (per core)
  - Active tasks and their runtime stats
- Stores info in structured internal format

### `Monitor`
- Manages system telemetry and web dashboard
- Intended for:
  - Wi-Fi web dashboard with real-time updates
  - Debug and diagnostics over HTTP/serial
- Provides JSON endpoints for telemetry data

## Class Diagram

```mermaid
classDiagram
    class ComponentBase {
        <<digitoys::core>>
        +initialize() esp_err_t
        +start() esp_err_t
        +stop() esp_err_t
        +shutdown() esp_err_t
        +getState() ComponentState
        +getName() const char*
    }

    class SystemMonitor {
        -uint32_t prev_idle_time_
        -uint32_t prev_total_time_
        +update() void
        +getFreeHeap() float
        +getMinHeap() float
        +getCpuUsage(int core) float
        +stats_get_handler(httpd_req_t*) esp_err_t
    }

    class Monitor {
        -Telemetry data_
        -httpd_handle_t server_
        -SemaphoreHandle_t mutex_
        +updateTelemetry(bool, float, float, bool) void
        +telemetry_get_handler(httpd_req_t*) esp_err_t
        +index_get_handler(httpd_req_t*) esp_err_t
        -init_wifi() esp_err_t
        -start_http_server() esp_err_t
        -stop_http_server() esp_err_t
    }

    class Telemetry {
        +obstacle : bool
        +warning : bool
        +distance : float
        +speed_est : float
    }

    ComponentBase <|-- SystemMonitor
    ComponentBase <|-- Monitor
    Monitor --> SystemMonitor
    Monitor --> Telemetry
```

---

## Public API

### `Monitor::Monitor()`

**Description:**  
Creates a Monitor component. Inherits from `ComponentBase` for standardized lifecycle management and registers with the centralized logging system.

---

### `Monitor::initialize()`

**Description:**  
Initializes the Monitor component (ComponentBase interface). Creates mutex for telemetry data protection.

**Returns:**  
- `ESP_OK` on success, error code on mutex creation failure

---

### `Monitor::start()`

**Description:**  
Starts the Monitor component including WiFi and HTTP server. Sets component state to RUNNING.

**Returns:**  
- `ESP_OK` on success, error code on WiFi/HTTP server failure

---

### `Monitor::stop()`

**Description:**  
Stops the Monitor component and HTTP server. Sets component state to STOPPED.

**Returns:**  
- `ESP_OK` on success, error code otherwise

---

### `Monitor::shutdown()`

**Description:**  
Completely shuts down the Monitor component. Sets component state to UNINITIALIZED.

**Returns:**  
- `ESP_OK` on success, error code otherwise

---

### `Monitor::updateTelemetry(bool obstacle, float distance, float speed_est, bool warning)`

**Description:**  
Updates telemetry data with current system state information.

**Parameters:**  
- `obstacle`: Emergency brake state
- `distance`: Current obstacle distance
- `speed_est`: Estimated vehicle speed
- `warning`: Progressive slowdown state

---

### `SystemMonitor::SystemMonitor()`

**Description:**  
Creates a SystemMonitor component. Inherits from `ComponentBase` and registers with centralized logging.

---

### `SystemMonitor::initialize()`

**Description:**  
Initializes the SystemMonitor component (ComponentBase interface).

**Returns:**  
- `ESP_OK` on success, error code otherwise

---

### `SystemMonitor::start()`

**Description:**  
Starts the SystemMonitor component. Sets component state to RUNNING.

**Returns:**  
- `ESP_OK` on success, error code otherwise

---
