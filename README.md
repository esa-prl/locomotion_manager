# Locomotion Manager

## Overview

This package implements a state machine that processes requests for changing the locomtion mode and handles the transitions. 

**Keywords:** locomotion, library, package

### License

The source code is released under a [TODO: Add License]().

**Author: Maximilian Ehrhardt<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Maximilian Ehrhardt, maximilian.ehrhardt@esa.int**

The locomotion_manager package has been tested under [ROS2] Eloquent and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rover_msgs] (message definitions for ESA-PRL rovers)

#### Building

To build from source, clone the latest version from this repository into your ros workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/locomotion_manager.git
	cd ../
	colcon build

## Usage

Run the main node with:

    ros2 run locomotion_manager locomotion_manager_node --ros-args --params-file src/locomotion_manager/config/locomotion_manager.yaml
## Config files

Config file config/

* **locomotion_manager.yaml** 
	- **`locomotion_modes`** List of existing locomotion modes. - `["simple", "crabbing", "spot_turn"]`

## Nodes

### locomotion_manager

This node contains all the logic.

#### Service Servers

* **`change_locomotion_mode`** ([rover_msgs/ChangeLocomotionMode])

	Receives a request to change the locomotion mode.

#### Service Clients

* **`/locomotion_mode_name/enable`** ([std_srvs/Trigger])

    Send a request to enable the locomotion mode with the given name.

* **`/locomotion_mode_name/disable`** ([std_srvs/srv/Trigger])

    Sends a request to disable the locomotion mode with the given name.

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
[rover_msgs]: https://github.com/esa-prl/rover_msgs
[rover_config]: https://github.com/esa-prl/rover_config.git
[rviz]: http://wiki.ros.org/rviz
[geometry_msgs/Twist]: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[sensor_msgs/JointState]: http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
[rover_msgs/JointCommandArray]: https://github.com/esa-prl/rover_msgs/blob/master/msg/JointCommandArray.msg