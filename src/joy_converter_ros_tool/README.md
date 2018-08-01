# Joy-Message to MotorCommand-Message Converter

This project contains the adapter-node `joy_converter_ros_tool`, which converts [sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html) to motor_interface_ros_tool/MotorCommand.

## Usage

Launch: `roslaunch joy_converter_ros_tool joystick.launch`

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request :D

## Nodes
### joy_converter_ros_tool
#### Parameters
  * `velocity_joystick_axis`(int): analoque-stick axis for velocity.
  * `steering_joystick_axis`(int): analoque-stick axis for steering angle.
  * `activation_joystick_button`(int): button for activate motor_interface_ros_tool.
  * `deactivation_joystick_button`(int): button for deactive motor_interface_ros_tool.
  * `max_velocity`(double): maximal velocity to publish.
  * `max_steering_angle`(double): maximal steering angle to publish.

#### Subscribtions
  * `joy`([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)): the joystick commands.

#### Publications
  * `motor_command`(motor_interface_ros_tool/MotorCommand): the servo-command
  * `servo_command`(motor_interface_ros_tool/ServoCommand): the servo-command

## History

TODO: Write history

## Credits

TODO: Write credits

## License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
