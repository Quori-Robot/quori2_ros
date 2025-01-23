# quori2_ros

This repo provides the ROS2 Humble packages for the Quori2 Robot.


## Building and using the packages

### dependencies

install additional ros-humble binaries that may not already be installed:

```
sudo apt-get install ros-humble-rplidar-ros ros-humble-teleop-twist-keyboard
```

You will need to install the OrbbecSDK_ROS2 source package into the same workspace src folder as quori2_ros

```
cd <ros_ws>/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

cd ..
```

To use the package git clone the repo to the ROS2 workspace/src folder.

Note: on the embedded Nvidia Orin AGX this should be done within the quori docker image that is run on the machine.

```
cd <ros_ws>/src
git clone git@github.com:Quori-Robot/quori2_ros.git

cd ..

colcon build --symlink-install
source .install/setup.bash
```

The Quori2 ROS application can be launched with the following command:


On the robot:

```
ros2 launch quori_launch quori_launch.py [mode:=navigation|mapping]
```

For Simulation:

```
ros2 launch quori_launch quori_sim_launch.py [mode:=navigation|mapping]
```


Using the Joystick:

Two types of joystick are currently supported - xbox360 and xbox_one

