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

The Quori2 ROS application can be launched with the following commands. There are also three helper scripts that can be used:
```
run_robot
run_sim
save_map
```

Thes scripts take the same arguments as shown below for the full ros2 launch commands.

These files are in the quori_launch/scripts folder, add this folder to your PATH env variable so they can be run from any folder.

*Run the robot*:

```
ros2 launch quori_launch quori_launch.py [mode:=navigation|mapping] [map:=<full path to map file>] [controller_type:=xbox360|xbox_one]
or
run_robot [mode:=navigation|mapping] [map:=<full path to map file>] [controller_type:=xbox360|xbox_one]
```

*Run the simulation*:

```
ros2 launch quori_launch quori_sim_launch.py [mode:=navigation|mapping] [map:=<full path to map file>] [controller_type:=xbox360|xbox_one] [world:=<full path to gazebo world file to use>]
or
run_sim [mode:=navigation|mapping] [map:=<full path to map file>] [controller_type:=xbox360|xbox_one] [world:=<full path to gazebo world file to use>]
```

Note: the map parameter is only valid if mode:=navigation

If the mode parameter is not give, neither mapping nor localization functions are started.

*Saving the map*:

When running in mapping mode to save the map - open a second terminal and run the save_map command

```
ros2 run nav2_map_server map_saver_cli -f <full path to map file name>
or
save map <map_name>
```

Note: when using the save_map helper script only give a short name for the map file (no path or extensions), it will automaticaly be saved to the quori_nav/maps folder.

Note: by default maps are expected to be in the quori_nav/maps folder

*Setting the map to use*:

After saving the map from mapping mode using one of the commands above, the map files (.png and .yaml) must be in the quori_nav/maps folder.

To use the map in navigation mode pass the map path/name to the launch command via the map parameter or edit the default value of the map parameter in the quori_lunch/launch/quori_launch.py or quori_lunch/launch/quori_sim_launch.py files.


### Using the Joystick

Two types of joystick are currently supported - xbox360 and xbox_one

Pass the controller_type parameter to select the joystick or set the default type of controller you are using by editing the default value of the 'controller_type' parameter in the quori_lunch/launch/quori_launch.py and quori_lunch/launch/quori_sim_launch.py launch files.

The button mapping is shown below

![XBox Mapping](https://github.com/user-attachments/assets/af197eff-d3b4-4bb6-bba7-c9376d8342a0)

### Using the keyboard

To operate the robot via the keyboard open a second terminal and run the teleop keyboard node as follows:

```
ros2 launch quori_launch teleop_kb_only_launch.py
```

Note: the keyboard teleop only alows controll of the robot base, not the head/neck joints.





