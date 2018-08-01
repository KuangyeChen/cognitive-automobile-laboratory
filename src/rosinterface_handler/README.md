# rosinterface_handler

## Package Summary
A unified parameter handler for nodes with automatic code generation.
Save time on defining your parameters, publishers and subscribers. No more redundant code. Easy error checking. Make them dynamic with a single flag.

- Maintainer status: maintained
- Maintainer: Fabian Poggenhans <fabian.poggenhans@fzi.de>
- Author: Claudio Bandera <cbandera@posteo.de>, Fabian Poggenhans <fabian.poggenhans@fzi.de>
- License: BSD


## Unified Interface Handling for ROS
When working with ROS, there usually a lot unneccessary and error-prone boilerplate code involved when writing new nodes. There are also a couple of tools to to be handled correctly, e.g. [Parameter Server](http://wiki.ros.org/Parameter%20Server) and [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure/).

With the multitude of options on where to specify your parameters and how to configure your publishers and subscribers, we often face the problem that we get a redundancy in our code and config files.

The `rosinterface_handler` let's you:
- specify your whole interface to the ROS world in a single file
- automatically create publishers and subscribers
- use a generated struct to hold your parameters
- use member method for grabbing the parameters from the parameter server
- use member method for updating them from dynamic_reconfigure.
- make your parameters configurable with a single flag.
- set default, min and max values
- choose between global and private namespace
- save a lot of time on specifying your parameters in several places.
- ...in both C++ and Python

## Usage
See the Tutorials on
- [How to write your first .rosif file](doc/HowToWriteYourFirstInterfaceFile.md)
- [How to use your parameter struct](doc/HowToUseYourInterfaceStruct.md)

## Installation
Download and build this package in your catkin workspace.

## Credits
This project is built upon a fork from the great [rosparam_handler](https://github.com/cbandera/rosparam_handler).
