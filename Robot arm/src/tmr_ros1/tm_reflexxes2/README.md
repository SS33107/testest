# tm_reflexxex2 : Advanced reflexxes package
This is an advanced package of tm_reflexxes with safety control APIs.

## Usage
- ```tm_action``` : send motion command to control tm5.
```
rosrun tm_reflexxes2 tm_action 192.168.0.10
```
- Launch with ```roslaunch tm_driver tm_bringup robot_ip:=192.168.0.10``` to launch rviz and robot model
- ```tm_otg``` : simply just run simulation using reflexxes motion library.

## TODOs
1. Subscribe the topic from ```close_kinect_tracking```node.
2. Now the safety API are still inside ```tm_action.cpp```, move to ```tm_reflexxes.cpp``` as library in the future.
