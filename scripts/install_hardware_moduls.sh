#!/bin/bash

if [ $(id -u) -ne 0 ]
then
    echo "This script must be run as root";
    exit 1;
fi

if modinfo uvcvideo | grep -q "version:"; then
    if modinfo uvcvideo | grep -q "realsense"; then
        echo "The required RealSenseSDK modules are already installed"
    else
        # based on https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
        sudo mkdir -p /etc/apt/keyrings
        curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
        sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
        echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list
        sudo apt-get update && sudo apt-get install -y \
            librealsense2-dkms \
            librealsense2-utils \
            librealsense2-dev \
            librealsense2-dbg
    fi
else
    echo "Could not get information about installed RealSenseSDK modules"
fi

# YDLidarSDK
# SDK_config_file="/usr/local/lib/cmake/ydlidar_sdk/ydlidar_sdkConfig.cmake"

# if [ -f "$SDK_config_file" ]; then
#     echo "The YDLidar SDK is already installed"
# else
#     # based on https://www.ydlidar.com/Public/upload/files/2022-06-21/YDLIDAR%20X2%20Lidar%20User%20Manual%20V1.3(211228).pdf
#     git clone https://github.com/YDLIDAR/YDLidar-SDK.git
#     mkdir -p YDLidar-SDK/build
#     cd YDLidar-SDK/build
#     curl -o initenv.sh https://raw.githubusercontent.com/YDLIDAR/ydlidar_ros_driver/master/startup/initenv.sh && \
#         chmod +x initenv.sh && \
#         sh initenv.sh
#     cmake ..
#     make
#     sudo make install
#     cd ../..
#     rm -rf YDLidar-SDK
# fi

