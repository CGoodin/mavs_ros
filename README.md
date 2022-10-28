# MAVS-ROS Package
Package to interface with ROS. 

## Requirements
Ubuntu 16.04 with [ROS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and a functioning [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) are required to build and run this code. It **may** work with more recent releases of Ubuntu but has not been tested.

## Installation
To run an example simulation with MAVS, first [install and build MAVS](https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator/-/wikis/MavsBuildInstructions).

Next, clone the example MAVS simulation package.
```bash
$git clone https://github.com/CGoodin/mavs_ros.git
```

To build the repo, from the top level catkin_ws directory, type
```bash
$catkin_make install
```

## Running the Example

To use the MAVS example, you will need to specify which scene file and vehicle file to use. This requires you to edit lines 3-4 of "mavs_sim.launch" to have the correct path to the data folder of your MAVS installation. With this complete, you can run the example.
```bash
$roslaunch mavs_ros_example mavs_sim.launch
```

The example does not have integrated autonomy. It can be used with the [NATURE software stack](https://github.com/CGoodin/nature-stack) to provide autonomy.
