# Project Name

motor_interface contains functionality to communicate with the motors on the anicars.

## Installation

For the usage of this library, access to `/dev/usb/iowarrior*` is needed. These devices are created by the kernel with root access permissions as soon as the motor interface is plugged in. To set access permissions for user group dialout copy `res/90-io-warrior.rules` to `/etc/udev/rules.d`. The user using applications relying on this library will need to be in the dialout user-group.

## Usage

TODO: Write usage instructions

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
