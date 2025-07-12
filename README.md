# cat_nav

This repository is under development. Release expected August 2025.


cat_nav is a package inspired by research on the desert ant Cataglyphis.

You drive the mobile robot around a course while the robot records regular visual snapshots. Now you can place the robot at the beginning of the course and the robot will follow the course. It uses the collection of snapshots to identify the closest snapshot and make minor course corrections to keep the robot on course.

This type of task has been termed Visual Teach & Repeat (VT&R) (Tim Barfoot).

## Installation

You should setup and activate your ROS2 environment. You may use any valid method, I use Robostack [RoboStack](https://robostack.github.io/GettingStarted.html)

```
cd ~/ros2_ws
git -C src clone https://github.com/jfrancis71/ros2_cat_nav.git
colcon build --symlink-install --packages-select=cat_nav
source ./install/setup.bash
```

## Recording A Route

```ros2 run cat_nav record_route --ros-args -p route_folder:=~/datasets/cat_nav/sunny_27_07_10_32```

The above command will create the route folder. You should then drive your robot around your desired route. When you have finished, Ctrl-C the program to stop it. There should now be a sequence of images in the route folder.

## Follow Route

```ros2 run cat_nav follow_route --ros-args -p route_folder:=~/datasets/cat_nav/sunny_27_07_10_32```

The above command will cause the robot to drive along the route. It will broadcast TwistStamped messages on the /cmd_vel topic. It moves primarily forwards with small angular components to keep the robot on course. It will stop if the image it is receiving is too different from any of the snapshots it has stored. This is effectively a 'I am lost' state, and it stops to prevent damage. You can physically move the robot and as soon as it recognises a snapshot it will continue.

