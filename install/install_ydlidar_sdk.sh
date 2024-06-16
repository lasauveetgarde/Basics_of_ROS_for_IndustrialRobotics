SDK_config_file="/usr/local/lib/cmake/ydlidar_sdk/ydlidar_sdkConfig.cmake"

# Проверяем, существует ли файл
if [ -f "$SDK_config_file" ]; then
    echo "Файл $SDK_config_file существует. SDK YDLidar уже установлен."
else
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    mkdir -p YDLidar-SDK/build
    cd YDLidar-SDK/build
    cmake ..
    make
    sudo make install
    cd ../..
    rm -rf YDLidar-SDK
fi