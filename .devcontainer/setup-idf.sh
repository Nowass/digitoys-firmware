#!/bin/bash
set -e

# Configurable
IDF_VERSION="v5.1"
IDF_PATH="/opt/esp/idf"
TOOLS_PATH="/opt/esp"

# Clone IDF if not already present
if [ ! -d "$IDF_PATH" ]; then
  echo "Cloning ESP-IDF $IDF_VERSION..."
  git clone --recursive --branch "$IDF_VERSION" https://github.com/espressif/esp-idf.git "$IDF_PATH"
fi

# Install the tools
echo "Running install.sh..."
cd "$IDF_PATH"
IDF_TOOLS_PATH="$TOOLS_PATH" ./install.sh

# Add export to bashrc if missing
if ! grep -q 'source /opt/esp/idf/export.sh' ~/.bashrc; then
  echo 'source /opt/esp/idf/export.sh' >> ~/.bashrc
fi
