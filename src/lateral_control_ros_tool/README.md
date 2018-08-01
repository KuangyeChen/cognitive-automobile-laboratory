# lateral_control_ros_tool

This package provides a lateral controller to make anicar follow a given track/path. The "Werling-Controller" is implemented which is one of the subjects taught in the lecture "Verhaltensgenerierung für  Fahrzeuge".

## Usage

Just launch
  roslaunch lateral_control_ros_tool lateral_controller_node.launch

The node is already configured in a way to interact with [motor_interface](https://gitlab.mrt.uni-karlsruhe.de/kognitive_automobile_labor/motor_interface_ros_tool).

## Nodes

### lateral_controller

#### Subscriptions
  * `path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)): the path to follow.

#### Publications
  * `servo_command` (motor_interface_ros_tool/ServoCommand): the command to the servo.


#### Control parameters
  * kos_shift : virtual shift of vehicle point along the vehicle x-axle.

  * index_shift: Index shift for lookahead point. This is the point the control calulates for. 

  * ii_off: point offset for curvature approximation.

  * k_ang: scaling angle influence.

  * k_dist: scaling distance influence.



## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## History

TODO: Write history

## Credits

TODO: Write credits
