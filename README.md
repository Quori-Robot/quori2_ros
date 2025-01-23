# quori2_ros

This repo provides the ROS2 Humble packages for the Quori2 Robot.


## Building and using the packages

### Dependencies

Install additional ros-humble binaries that may not already be installed:

```
sudo apt-get install ros-humble-rplidar-ros ros-humble-teleop-twist-keyboard  ros-humble-ros-gz
```

You will need to install the OrbbecSDK_ROS2 source package into the same workspace src folder as quori2_ros

```
cd <ros_ws>/src
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

cd ..
```


### Clone quori2_ros

Git clone the quori2_ros repo to the ROS2 workspace/src folder.


```
cd <ros_ws>/src
git clone git@github.com:Quori-Robot/quori2_ros.git

cd ..
```

### Build the packages

Note: on the embedded Nvidia Orin AGX this should be done within the quori docker image that is run on the machine.

```
cd <ros_ws>/src
colcon build --symlink-install
source .install/setup.bash
```


### Run the robot

The Quori2 ROS application can be launched with the following command:


On the robot:

```
ros2 launch quori_launch quori_launch.py [mode:=navigation|mapping] [map:=<full path to map file>] [controller_type:=xbox360|xbox_one]
```

For Simulation:

```
ros2 launch quori_launch quori_sim_launch.py [mode:=navigation|mapping] [map:=<full path to map file>] [controller_type:=xbox360|xbox_one] [world:=<full path to gazebo world file to use>]
```

Note: the map parameter is only valid if mode:=navigation



### Using the Joystick:

Two types of joystick are currently supported - xbox360 and xbox_one

Set the type of controller you are using be editing the default value of the 'controller_type' parameter in teh quori_launch.py and quori_sim_launch.py launch files

### Using the keyboard:



