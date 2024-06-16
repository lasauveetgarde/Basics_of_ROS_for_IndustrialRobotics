<!-- omit from toc -->
# Установка всего-всего необходимого 

## Содержание 

- [Содержание](#содержание)
- [Как установить ROS?](#как-установить-ros)
- [Как установить пакеты ROS?](#как-установить-пакеты-ros)
- [Подготовка turtlebot3](#подготовка-turtlebot3)
- [YDLIdar ROS Driver](#ydlidar-ros-driver)

## Как установить ROS?

Установка ROS описана [на официальном сайте](http://wiki.ros.org/noetic/Installation/Ubuntu).

Не забываем после установки активировать **системное рабочее пространство**. Для этого нужно выполнить в терминале 2 команды:

```bash
# Активация ROS при каждом запуске терминала
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# Активация ROS в этом терминале
source ~/.bashrc
```

Если используется в качестве оболочки zsh, то пользуется немного другим `setup` файлом:

```bash
# Активация ROS при каждом запуске терминала
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
# Активация ROS в этом терминале
source ~/.zshrc
```

## Как установить пакеты ROS?

Вот такой командой мы устанавливаем пакеты в Ubuntu:

```bash
sudo apt install [имя пакета]
```

Для ROS, например, пакеты имеют шаблон `ros-noetic-[название]`. Например, для установки `move-base` вызываем команду: 

```bash
sudo apt install ros-noetic-move-base
```

## Подготовка turtlebot3

Первоначально все необходимые для теста пакета ставятся следующим образом:

```bash
sudo apt install \
    ros-noetic-gmapping \
    ros-noetic-dwa-local-planner \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-teleop \
    ros-noetic-turtlebot3-slam \
    ros-noetic-turtlebot3-navigation \
    ros-noetic-teleop-twist-joy 
```

## YDLIdar ROS Driver

``` sh
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
```


