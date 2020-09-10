# Package Name

## Overview

This package provides a ROS 2 lifecycle node to interface with the [platform_driver_ethercat] library for accessing Elmo drives and ATI force torque sensors via EtherCAT.

**Keywords:** ati fts, elmo, ethercat, joint control, soem

### License

The source code is released under a [GPLv3 License](https://www.gnu.org/licenses/gpl-3.0.en.html).

**Author: Tim Wiese<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Tim Wiese, tim.wiese@esa.int**

This package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [platform_driver_ethercat] (library for Elmo Gold Twitter and ATI FTS communication over EtherCAT using [SOEM]
- [rover_msgs](https://github.com/esa-prl/rover_msgs) (message definitions for ESA-PRL rovers)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) (a YAML parser and emitter in C++)

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/platform_driver_ethercat_ros2.git
	cd ../
	colcon build

## Usage

Since the [platform_driver_ethercat] library uses [SOEM] which needs raw socket access, the node needs to be run with root privileges. This could be circumvented in the future with `setcap` and https://github.com/ros2/rcpputils/pull/44.

Change into a root shell while preserving your environment with

    sudo -sE

Then launch the main node with

	ros2 launch platform_driver_ethercat_ros2 platform_driver_ethercat.launch.py pd_config_file:=/path/to/config/file

## Launch files

launch/

* **platform_driver_ethercat.launch.py** Minimal launch file to start the node and bring it to the `Active` state. Needs to be executed with root privileges.

    Arguments:

    - **`pd_config_file`** Full path to the configuration file

## Config files

config/

* **pd_marta.yaml** Configuration file for the MaRTA rover, to configure SOEM, Elmo drives and joints. This is **NOT** a regular ROS 2 parameter file. It makes use of advanced yaml features and is parsed inside the node.

## Nodes

### platform_driver_ethercat_node

Lifecycle node to interface with the [platform_driver_ethercat] library.

#### Subscribed Topics

* **`joint_cmds`** ([rover_msgs/JointCommandArray](https://github.com/esa-prl/rover_msgs))

	Commanded position, velocity or torque for joints.

#### Published Topics

* **`joint_states`** ([sensor_msgs/JointState](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html))

	Measured position, velocity and effort of joints.

* **`fts_readings`** ([rover_msgs/WrenchStampedArray](https://github.com/esa-prl/rover_msgs))

	Measured force and torque values of sensors

* **`temp_readings`** ([rover_msgs/TemperatureArray](https://github.com/esa-prl/rover_msgs))

	Measured temperatures in degree Celsius.

#### Parameters

* **`config_file`** (string)

	Full path to the configuration file.

## Bugs & Feature Requests

Please report bugs and request features using the GitHub issue tracker.

[platform_driver_ethercat]: https://github.com/esa-prl/drivers-platform_driver_ethercat
[ROS2]: http://www.ros.org
[SOEM]: https://github.com/OpenEtherCATsociety/SOEM
