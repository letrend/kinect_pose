This is the [kinect_pose](https://www.youtube.com/watch?v=fngvU52N_P4 "kinect pose rviz plugin") project.
It's a rviz plugin for kinect_v2 and a cuda capable gpu. It uses [ICPCUDA](https://github.com/mp3guy/ICPCUDA) for the pose estimation. This is very fast, however rendering everything in rviz takes a while. 

## Dependencies
Please follow the installation instruction of ICPCUDA.
Additionally you will need either ros indigo or jade.

## Installation
```
#!/bin/bash

cd /path/to/kinect_pose
catkin_make
source devel/setup.bash
rviz
```
In rviz select Panels->AddPanel->KinectPosePlugin

