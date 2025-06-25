# Digitoys Telemetry Dashboard

This folder contains a minimal HTML dashboard that polls the `/telemetry` endpoint provided by the firmware's monitor component.

## Usage

1. Build and flash the firmware so the monitor's HTTP server is running on your ESP32-C6.
2. Copy the `dashboard` folder to any web server (or run `python3 -m http.server` inside it).
3. Open the page in a browser using the board's IP address (e.g. `http://<board-ip>/`).
4. The page will refresh the latest telemetry every second.

The same HTML page is also embedded in the firmware and served from the root URL.
