git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git
git ydlidar_ros_driver apply ../patches/ydlidar_ros_driver.patch
sudo chmod 0777 /src/ydlidar_ros_driver/startup/*
sudo sh src/ydlidar_ros_driver/startup/initenv.sh
