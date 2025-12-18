# ESP32 LED Mesh Firmware Build Environment
# Based on ESP-IDF v6.0-beta1
#
# Usage:
#   Build image:
#     docker build -t esp32-led-mesh-builder .
#
#   Build firmware for ESP32:
#     docker run --rm -v $(pwd):/project esp32-led-mesh-builder esp32
#
#   Build firmware for ESP32-C3:
#     docker run --rm -v $(pwd):/project esp32-led-mesh-builder esp32c3
#
#   Build for all targets:
#     docker run --rm -v $(pwd):/project esp32-led-mesh-builder all
#
#   Interactive shell:
#     docker run --rm -it -v $(pwd):/project esp32-led-mesh-builder shell

FROM espressif/idf:v6.0-beta1

# Install any additional packages needed
RUN apt-get update && apt-get install -y \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Create build script
COPY docker-build.sh /usr/local/bin/build.sh
RUN chmod +x /usr/local/bin/build.sh

# Set working directory for projects
WORKDIR /project

# Default entrypoint runs the build script
ENTRYPOINT ["/usr/local/bin/build.sh"]
CMD ["all"]
