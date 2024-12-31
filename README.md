# quori2_ros

This repo provides the ROS2 Humble packages for the Quori2 Robot.


## Building and using the packages
To use the packages git clone the repo to the ROS2 workspace/src folder.

Note: on the embedded Nvidia Orin AGX this should be done within the quori docker image that is run on the machine.


```
cd <ros2_ws>/src
git clone git@github.com:Quori-Robot/quori2_ros.git

cd ..

colcon build --symlink-install
source .install/setup.bash
```

The Quori2 ROS application can be launched with the following command:
```
ros2 launch quori_launch quori_launch.py
```


