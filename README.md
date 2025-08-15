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

In the below step you will presumably want to change my name (julian) for your username. Also I have followed a convention of naming the folder with the lighting conditions, date and time of day. (day of month, month, hour, minutes). You can use whatever naming convention you find helpful. Also I have renamed the image subscription topic, you should use whatever topic name your robot is publishing images on:

```ros2 run cat_nav record_route_node --ros-args -p route_folder:=/home/julian/datasets/cat_nav/sunny_27_07_10_32 -r /image:=/kitt/image```

The above command will create the route folder. You should then drive your robot around your desired route. When you have finished, Ctrl-C the program to stop it. There should now be a sequence of images in the route folder.

#### Parameters

|Name|Default|Description|
|----|-------|-----------|
|route_folder|./default_route_folder|Path to folder to store sequence of route images|
|record_interval|.05|Record image every record_interval metres|

#### Subscribed Topics

|Topic Name|Message Type|Description|
|----------|------------|-----------|
|/image|Image|Camera image|
|/odom|Odometry|Odometry information|

#### No Published Topics

## Follow Route

You should replace the route folder given below with the folder path you have a recorded route for, and change the image topic name to whatever your robot is publishing on:

```ros2 run cat_nav follow_route_node --ros-args -p route_folder:=/home/julian/datasets/cat_nav/sunny_27_07_10_32 -r /image:=/kitt/image```

The above command will cause the robot to drive along the route. It will broadcast TwistStamped messages on the /cmd_vel topic. It moves primarily forwards with small angular components to keep the robot on course. It will stop if the image it is receiving is too different from any of the snapshots it has stored. This is effectively a 'I am lost' state, and it stops to prevent damage. You can physically move the robot and as soon as it recognises a snapshot it will continue.

#### Parameters

|Name|Default|Description|
|----|-------|-----------|
|route_folder|./default_route_folder|Path to folder to sequence of route images|
|route_loop|False|If False we stop when we have completed route. If True then if route is a loop we continue at beginning|
|lost_edge_threshold|450|Threshold at which we consider ourselves "lost" and stop the robot|
|self_drive|True|If this is False do not issue drive commands, useful for debugging|
|lost_seq_len|5|Number of consecutive "lost" states we have received before considering ourselves actually lost|
|warning_time|.25|Maximum time allowed from camera time stamp message to issue drive command before issuing warning|
|publish_diagnostic|True|Publish diagnostic image|
|angle_ratio|36.|We divide the side pixel error by this ratio to give the angular z velocity command (to correct trajectory)|
|stop_on_last|5|stop if we are within stop_on_last of last image frame|
|forward_speed|.05|forward speed of robot|


#### Subscribed Topics

|Topic Name|Message Type|Description|
|----------|------------|-----------|
|/image|Image|Camera image|

#### Published Topics

|Topic Name|Message Type|Description|
|----------|------------|-----------|
|/cmd_vel|TwistStamped|Robot movement commands|


# Inspiration

Barbara Webb, Modelling the learning circuits of the insect brain, YouTube, CRISP - Queen Mary University of London. 04 Dec 2022
https://www.youtube.com/watch?v=4bde8rNU3F4&t=1339s

A Model of Ant Route Navigation Driven by Scene Familiarity, Baddeley B, Graham P, Husbands P, Philippides A.
https://journals.plos.org/ploscompbiol/article?id=10.1371/journal.pcbi.1002336

Paul Graham, Ant Navigation: How To Be A Champion Navigator With A Small Brain, YouTube, Biological Recording Company, 07 Sep 2023
https://www.youtube.com/watch?v=0TxiBBYQxms&t=2430s

Continuous Visual Navigation with Ant Inspired Memories, YouTube, Gabriel, 09 Dec 2024
https://www.youtube.com/watch?v=Osu5Jyy6dF4&t=268s

Barbara Webb, Neural Mechanisms of Insect Navigation, YouTube, Stanford, 17 Nov 2017
https://www.youtube.com/watch?v=KpaPakvEGkY&t=1666s

A Neural Data Structure for Novelty Detection, Dasgupta et al
Makes connections with Bloom filters (from mushroom body)
https://www.pnas.org/doi/10.1073/pnas.1814448115

Sensory encoding and memory in the mushroom body: signals, noise, and variability
Moshe Parnas, Julia E Manoim, Andrew C Lin
https://pmc.ncbi.nlm.nih.gov/articles/PMC11199953/

An intrinsic oscillator underlies visual navigation in ants
Leo Clement 1 2, Sebastian Schwarz 1, Antoine Wystrach
https://www.sciencedirect.com/science/article/pii/S0960982222018504

Looking and homing: how displaced ants decide where to go
Zeil, Narendra, Sturzi (2014)
https://pmc.ncbi.nlm.nih.gov/articles/PMC3886322/

Visual cues for the retrieval of landmark memories by navigating wood ants (2007)
Harris RA, Graham P, Collett TS.
https://pubmed.ncbi.nlm.nih.gov/17240334/

MIT Robotics – Tim Barfoot - Robot Navigation When Lighting, Weather, and Geometry Won’t Cooperate
Tim Barfoot (2024)
https://www.youtube.com/watch?v=oelXylttvS4&t=2618s

