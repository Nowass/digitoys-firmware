# 🚀 ESP32-C6 Firmware Development with VSCode Dev Container

<!-- vscode-markdown-toc -->

- [📦 Prerequisites](#Prerequisites)
- [🧭 First-Time Setup (One-Time per Machine)](#First-TimeSetupOne-TimeperMachine)
- [🛠️ Working with the ESP-IDF Extension](#WorkingwiththeESP-IDFExtension)
  - [🧱 Build the Project](#BuildtheProject)
  - [🔥 Flash the Firmware to the ESP32-C6 (JTAG)](#FlashtheFirmwaretotheESP32-C6JTAG)
  - [📟 Monitor Serial Output](#MonitorSerialOutput)
  - [🐞 Start Debugging (JTAG)](#StartDebuggingJTAG)
- [🔄 Reopening the Dev Container](#ReopeningtheDevContainer)
- [✅ What’s Preconfigured](#WhatsPreconfigured)
- [📁 Create a New Component Folder](#CreateaNewComponentFolder)
- [✍️ Implement the Component](#ImplementtheComponent)
- [⚙️ Add a `CMakeLists.txt` for the Component](#AddaCMakeLists.txtfortheComponent)
- [📦 Add the Component to the Project](#AddtheComponenttotheProject)
- [🧪 Use the Component in Your Application](#UsetheComponentinYourApplication)
- [🔄 Reconfigure & Build](#ReconfigureBuild)

<!-- vscode-markdown-toc-config
	numbering=false
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->

This project uses **Visual Studio Code Dev Containers** and the **ESP-IDF extension** to provide a fully isolated, reproducible development environment for ESP32-C6 firmware development — without polluting your host system.

Everything (toolchain, Python, OpenOCD, and IDF) is preinstalled in a Docker container that launches automatically through VSCode.

## <a name='Prerequisites'></a>📦 Prerequisites

Make sure the following are installed on your machine:

| Tool                         | Link                                                                                   |
| ---------------------------- | -------------------------------------------------------------------------------------- |
| **Visual Studio Code**       | https://code.visualstudio.com/                                                         |
| **Docker Desktop / Engine**  | https://www.docker.com/products/docker-desktop                                         |
| **Dev Containers Extension** | https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers |

## <a name='First-TimeSetupOne-TimeperMachine'></a>🧭 First-Time Setup (One-Time per Machine)

1. **Clone this project**:

   ```bash
   git clone https://github.com/Nowass/digitoys-firmware
   cd digitoy-firmware
   ```

2. **Open the folder in VSCode**:

   ```bash
   code .
   ```

3. **When prompted**, choose:

   > `Reopen in Container`

   Or manually via Command Palette:
   `F1` → `Dev Containers: Reopen in Container`

   > In case the docker permission is missing fro the current user run he following:
   ```bash
   sudo usermod -aG docker $USER
   sudo reboot
   ```

4. **Wait for the container to build** (🚀 _First time may take a few minutes_)

   - Tools will be downloaded
   - Python environment + ESP-IDF will be initialized
   - This only happens **once** — future starts are fast

## <a name='WorkingwiththeESP-IDFExtension'></a>🛠️ Working with the ESP-IDF Extension

Once the Dev Container loads, the **ESP-IDF extension** is already installed and configured.

Use the **VSCode status bar and Command Palette (`F1`)** to access the most common operations:

### <a name='BuildtheProject'></a>🧱 Build the Project

- 📥 Click the **“Build”** button in the **status bar**
  or
- Press `F1` → type `ESP-IDF: Build your project`

This runs `idf.py build` behind the scenes and compiles your firmware.

### <a name='FlashtheFirmwaretotheESP32-C6JTAG'></a>🔥 Flash the Firmware to the ESP32-C6 (JTAG)

- 🔌 Connect your board to USB/JTAG
- 📦 Click **“Flash”** in the status bar
  or
- Press `F1` → `ESP-IDF: Flash your project`

> 💡 Choose `YES` to start the OpenOCD server if not running.

> 💡 The default serial port is usually `/dev/ttyUSB0` on Linux.

### <a name='MonitorSerialOutput'></a>📟 Monitor Serial Output

After flashing, open the serial monitor:

- Press `F1` → `ESP-IDF: Monitor your device`

> You can also combine **flash + monitor** using:
> `ESP-IDF: Flash (with monitor)`

By default, this project assumes your device appears as:

```json
"idf.port": "/dev/ttyUSB0"
```

> 🐧 **Linux users:** this is usually correct
> 🪟 **Windows users:** change this to e.g. `"COM3", "COM4", etc.`

To change it:

1. Open `.vscode/settings.json`
2. Update the `idf.port` value to match your platform and device

### <a name='StartDebuggingJTAG'></a>🐞 Start Debugging (JTAG)

1. Connect the board to USB/JTAG debugger
2. Press `F5` or go to the **Run and Debug** panel
3. Select the **“Launch”** debug configuration

## <a name='ReopeningtheDevContainer'></a>🔄 Reopening the Dev Container

If you ever want to reopen the project in the container after closing:

- `F1` → `Dev Containers: Reopen in Container`

To exit back to host:

- `F1` → `Dev Containers: Reopen Folder Locally`

## <a name='WhatsPreconfigured'></a>✅ What’s Preconfigured

- ESP-IDF v6.0 toolchain for ESP32-C6
- Python virtual environment with all required packages
- OpenOCD for JTAG/debugging
- JTAG flash tool
- Serial monitor
- Fully isolated and reproducible development environment

# 🧩 Adding a New Component

This project supports modular C++ development using **ESP-IDF components**. Components allow you to organize features or libraries (e.g. drivers, logic blocks) into reusable units.

## <a name='CreateaNewComponentFolder'></a>📁 Create a New Component Folder

Inside the project root, create the folder `components/my_component`:

```bash
mkdir -p components/my_component
```

**Folder Structure:**

```
components/
└── my_component/
    ├── MyComponent.cpp
    ├── MyComponent.hpp
    └── CMakeLists.txt
```

## <a name='ImplementtheComponent'></a>✍️ Implement the Component

**Example: `MyComponent.hpp`**

```cpp
#pragma once

class MyComponent {
public:
    static void do_something();
};
```

**Example: `MyComponent.cpp`**

```cpp
#include "MyComponent.hpp"
#include <stdio.h>

void MyComponent::do_something() {
    printf("MyComponent is doing something!\n");
}
```

## <a name='AddaCMakeLists.txtfortheComponent'></a>⚙️ Add a `CMakeLists.txt` for the Component

**`components/my_component/CMakeLists.txt`**

```cmake
idf_component_register(SRCS "MyComponent.cpp"
                       INCLUDE_DIRS ".")
```

ESP-IDF will detect the `.cpp` extension and use `CXX` to compile the component.

## <a name='AddtheComponenttotheProject'></a>📦 Add the Component to the Project

If your project doesn’t already have this in the top-level `CMakeLists.txt`, add it:

```cmake
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
```

> ⚠️ If `components/` already exists and you plan to have multiple components, just add the folder — not individual subfolders.

## <a name='UsetheComponentinYourApplication'></a>🧪 Use the Component in Your Application

**Example: `src/main.cpp`**

```cpp
#include "MyComponent.hpp"

extern "C" void app_main(void)
{
    MyComponent::do_something();
}
```

> 💡 The `extern "C"` around `app_main` is required because ESP-IDF looks for a **C-style symbol**.

## <a name='ReconfigureBuild'></a>🔄 Reconfigure & Build

**Via ESP-IDF Extension:**

- `F1` → `ESP-IDF: Reconfigure project`
- `F1` → `ESP-IDF: Build your project`

**Or from terminal:**

```bash
idf.py reconfigure
idf.py build
```

Happy hacking! ✨

_Powered by ESP-IDF, VSCode, and Dev Containers 🐳_
