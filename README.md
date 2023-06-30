# Kobuki ZED ORB SLAM3 Integration

This ROS package implements the ORB SLAM3 Integration using zed camera on Turtlebot2 Kobuki Model, where the package also includes ground truth recording function using ouster OS1 64 lidar with HDL graph slam. The embed board was tested on jetson xavier NX on [Jetpack 5.1](https://developer.nvidia.com/embedded/jetpack-sdk-51). 



### PC and Jetson Common Setup:

Install ROS Noetic

Setup workspace:

```bash
$ mkdir ~/catkin_ws/src -p
$ cd ~/catkin_ws
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ cd src
```

[Turtlebot2 on noetic](https://github.com/hanruihua/Turtlebot2_on_Noetic)

```bash
$ sudo apt-get install ros-noetic-sophus ros-noetic-joy libusb-dev libftdi-dev ros-noetic-base-local-planner ros-noetic-move-base-msgs pyqt5-dev-tools net-tools
$ git clone https://github.com/hanruihua/Turtlebot2_on_Noetic.git
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ~/.bahsrc
```

Clone this wrapper:

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/Seekerzero/kobuki_zed_orb_slam3.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ~/.bashrc
```





### Jetson Setup:

Install [ZED SDK 3.8  for L4T 35.1 (Jetpack 5.0)](https://www.stereolabs.com/developers/release/)

Install [ZED ROS wrapper](https://github.com/stereolabs/zed-ros-wrapper.git):

```bash
$ cd catkin_ws/src
$ git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
$ cd catkin_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ source ~/.bashrc
```

Setup udev rule for connection with kobuki:

```bash
$ rosrun kobuki_ftdi create_udev_rules
```





### Laptop Setup:

Setup [Ouster ROS Wrapper](https://github.com/ouster-lidar/ouster-ros):

```bash
$ sudo apt install -y ros-noetic-pcl-ros ros-noetic-rviz
$ sudo apt install -y build-essential libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev cmake net-tools
$ cd ~/catkin_ws/src
$ git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
$ cd ~/catkin_ws
$ catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
$ source ~/.bashrc
```

Setup the [network with OS1 lidar](https://static.ouster.dev/sensor-docs/image_route1/image_route2/networking_guide/networking_guide.html):

Get the hostname using:

```bash
avahi-browse -lrt _roger._tcp
```

and change the **sensor_hostname** in **lidar_sensor.launch** to the lidar hostname.



Setup the [FAST-LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM) Integration:

Setup the dependencies:

​	Install [ceres solver](http://ceres-solver.org/installation.html)

​	Install [GTSAM](https://gtsam.org/get_started/)

Install the ROS packages:

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/Livox-SDK/livox_ros_driver.git
$ git clone https://github.com/gisbi-kim/FAST_LIO_SLAM.git
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
$ source ~/.bashrc
```



Modify the ROS topic based on your lidar topics in fast_lio launch and yaml files.



### Setup the ROS Network:

Follow the instruction on section 1.1.6 of [Turtlebot3 PC setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) to setup the ROS connection between laptop and Jetson.



### Record data:

On the laptop:

```bash
$roscore
```

Connect the jetson to Internet to sync the time, otherwise two bags from jetson and laptop will not be able merge into same timestamps. And then connect the jetson to ROS network which also connect the laptop.

SSH to jetson board:

```bash
$ ssh jetson_name@ip
```

Bring up the kobuki:

```bash
$ roslaunch turtlebot_bringup minimal.launch
```

On a new ssh terminal, launch the zed node and the robot state publisher:

```bash
$ roslaunch kobuki_zed_orb_slam3 view_model.launch
```

On a new ssh terminal, rosbag the topics:

```bash
$ rosbag record -b 0 --split --duration=10m /camera/fisheye1/camera_info /camera/fisheye1/image_raw /camera/fisheye1/metadata /camera/fisheye2/camera_info /camera/fisheye2/image_raw /camera/fisheye2/camera/fisheye2/metadata /camera/imu /initialpose /joint_states /mobile_base/sensors/imu_data /odom /ouster/imu /ouster/metadata /ouster/points /ouster/range_image /tf /tf_static

```



On the laptop:

```bash
$ roslaunch kobuki_zed_orb_slam3 lidar_sensor.launch
```

rosbag the topics:

```bash
$ rosbag record /ouster/points /ouster/imu /ouster/range_image
```

control the kobuki with:

```bash
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```



### Processing Data

Install [rosbag-merge](https://pypi.org/project/rosbag-merge/):

```bash
$ pip3 install rosbag-merge
```

get into the folder where contains two bag files:

```bash
$ rosbag-merge --outbag_name out.bag
```

launch the fast_lio_slam node:

```bash
$ roslaunch roslaunch fast_lio mapping_ouster64.launch
```

play the merge bag:

```bash
$ rosbag play --clock out.bag
```

To view the robot model in the record data;

```bash
$ roslaunch kobuki_zed_orb_slam3 view_model.launch
```





### ORB SLAM3 Installation

Follow the instruction on [orb_slam3_ros](https://github.com/thien94/orb_slam3_ros) tosetup the ORB SLAM3 on ROS. 