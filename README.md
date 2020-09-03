# MAVS AVT-341 Example
MAVS package to interface with the AVT-341 autonomy algorithms for simulation.

## Requirements
Ubuntu 16.04 with [ROS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and a functioning [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) are required to build and run this code. It **may** work with more recent releases of Ubuntu but has not been tested.

## Installation
To run an example simulation with MAVS, first [install and build MAVS](https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator/-/wikis/MavsBuildInstructions).

Next, clone the example MAVS simulation package.
```bash
$git clone https://github.com/CGoodin/mavs_avt_example.git
```

In order to build the repo, you will need to modify the CMakeLists.txt file in the mavs_avt_example to find your MAVS installation. In lines 8-9, change the following lines 
```
SET(mavs_INCLUDE_DIR  "/home/msucavs/mavs/src/")
SET(mavs_LIB_DIR  "/home/msucavs/mavs/build/lib")
```
to match the path to the MAVS "src" and "lib" directories on your system. Then, from the top level catkin_ws directory, type
```bash
$catkin_make install
```

To use the MAVS example with the AVT-341 autonomy, uncomment lines 58-62 of "example.launch" and comment out line 52. Then, run the example as before.
```bash
$roslaunch mavs_avt_example mavs_sim.launch
```
