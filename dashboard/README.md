# Dashboard

This dashboard displays real-time telemetry data from the Digitoys ESP32 firmware.

## Usage

1. Build and flash the firmware with the monitor component enabled so that `/telemetry` is available over HTTP.
2. Open `index.html` in any modern web browser. Because the page can be loaded from anywhere, specify the ESP32 address using the `host` query parameter:

   ```
   file:///path/to/index.html?host=http://<device-ip>
   ```
   e.g. `file:///home/nowass/GIT/digitoys-firmware/dashboard/index.html?host=http://192.168.191.105`

   You may also serve the file from any local or remote web server in the same way:

   ```
   http://your-server/index.html?host=http://<device-ip>
   ```

The dashboard will poll `<host>/telemetry` every second and display the obstacle
status, distance and speed.