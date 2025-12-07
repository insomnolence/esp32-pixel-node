# ESP32 LED Mesh Firmware Build Environment
# Based on ESP-IDF v6.0 (master branch as of the required commit)
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

FROM ubuntu:22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install required system packages
RUN apt-get update && apt-get install -y \
    git \
    wget \
    flex \
    bison \
    gperf \
    python3 \
    python3-pip \
    python3-venv \
    cmake \
    ninja-build \
    ccache \
    libffi-dev \
    libssl-dev \
    dfu-util \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Set Python 3 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# ESP-IDF version - using v6.0 development branch
# This matches the version: v6.0-dev-1489-g4e036983a7 or later
ENV IDF_BRANCH=master
ENV IDF_COMMIT=v6.0-dev-1489-g4e036983a7

# Clone ESP-IDF
WORKDIR /opt
RUN git clone --recursive https://github.com/espressif/esp-idf.git esp-idf && \
    cd esp-idf && \
    git checkout ${IDF_COMMIT} && \
    git submodule update --init --recursive

# Install ESP-IDF tools for all supported targets
ENV IDF_PATH=/opt/esp-idf
ENV IDF_TOOLS_PATH=/opt/espressif

RUN cd /opt/esp-idf && \
    ./install.sh esp32,esp32c2,esp32c3,esp32s3

# Create build script
COPY docker-build.sh /usr/local/bin/build.sh
RUN chmod +x /usr/local/bin/build.sh

# Set working directory for projects
WORKDIR /project

# Default entrypoint runs the build script
ENTRYPOINT ["/usr/local/bin/build.sh"]
CMD ["all"]
