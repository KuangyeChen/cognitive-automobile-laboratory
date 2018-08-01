# Vehicle Setup (Meta-Package)

This meta-package contains launch-files to setup the basic functinality of the vehicle including:
  * cameras (with the correct frame_ids)
  * localization via stargazer
  * stereo image processing + view
  * geometrc vehicle model with coordinate system definitions

Also launch-files for visualization are included.

**Note:** This package does **not** contain launch-files for actuator access cause this would be a duplication to the launchfile provided by motor_interface_ros_tool. Also launch-files for manual motor control are not provided here but by thier respective package; e.g. tui_ros_tool or joy_to_motor_command_ros_tool.

## Usage

### Cameras

Every Camera has its own launch-file:
  * camera_front_left.launch
  * camera_front_right.launch
  * camera_top.launch

### Localization with stargazer
Just launch  ``roslaunch vehicle_setup localization.launch``.

#### Calibration ####
If the calibrations provied in this package are not correct anymore, a new calibration can be created easily in two steps:

1. Record a rosbag of the landmarks by executing ``roslaunch vehicle_setup record_stargazer_calib.launch`` and drive the car around so that every landmark has been seen by the camera from multiple poses.
2. Launch `` roslaunch vehicle_setup calibrate_landmarks.launch`` to perfom the actual calibration.

After this you can use the stargazer as usal with the new calibration.

**Note:** To backup your new calibration you should checkout a new branch and commit it. Also a merge request containing the new calibration would be appropriated so that others can use it as well.

**Note:** For proper visualization in rviz, you need to upload the newly estimated intrinsics to the top camera. This can be done by launching
  `` roslaunch vehicle_setup upload_top_camera_intrinsics.launch ``.

### Stereo
For Stereo processing launch ``roslaunch vehicle_setup stereo.launch``. This launches **only** the **processing** part, not the camera nodes!
For rectification and stereo-matching the [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) pipeline is used. For configuration see [this tutorial](http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters).
In order to reduce computational cost and increase robustness the input images are resized. The scaling factor can be set by the argument ``scale``.

For stereo view launch ``roslaunch vehicle_setup stereo_view.launch``.

**Note:** there are hard dependencies between these two launch files! If you change one of them, make sure it does not break the other.

### Vehicle Geometry And TFs
Just launch ``roslaunch vehicle_setup urdf_publisher.launch``.

### Visualization
Just launch ``roslaunch vehicle_setup start_visualization.launch``
This visualizes the landmarks, the vehicle with all its coordinate-system,  the point-cloud obtained by the stereo-matching and the driven trajectory in rviz.

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
