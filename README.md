# ros2_react
This repo contains submodules, so don't forget to init them:
- git submodule init
- git submodule update
--- 
Also you need to change the frames in the `slam_gmapping.cpp`:
```
base_frame_ = "robot/base_link";
map_frame_ = "map";
odom_frame_ = "robot/odom";
```
---
To run the package:
```
ros2 launch react_nav reactive.launch.py
```
There is also a teleop launch file:
```
ros2 launch react_nav teleop.launch.py
```
