# A Simple Reactive Robot

## Project Dependencies
This project was developed under Ubuntu 16.04 with ROS Kinetic, compatibility with older or newer versions is not guaranteed.

The project requires the following dependencies installed:

* ROS Kinetic and proper build tools installed
* A properly configured catkin workspace
* The STDR simulator packages installed in ROS
* A Python interpreter, version 2.7 or equivalent/compatible

## How to run this project
#### Installing the packages
1. Copy the directory containing this code into your catkin workspace's "src" folder.
2. Open a terminal, "cd" into the root of your workspace and run "catkin_make".

#### Launching the Server, GUI, Map and spawning robots:
Open a terminal and run the ```roslaunch ROBO <launcher>``` command, replacing ```<launcher>``` with one of the provided launchers.

**For example:** ```roslaunch ROBO server_with_map_gui_and_2_simplex.launch```

#### Launching the AI nodes of each robot
Open a terminal for each robot spawned and launch the nodes with the following command: ```rosrun ROBO avoid_wall.py <id>```

Replace ```<id>```with the respective id of a robot

**For example:** ```rosrun ROBO avoid_wall.py 0```



