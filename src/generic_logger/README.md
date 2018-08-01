# Generic Logger
Minimalistic logger interface supporting ROS-agnostic library development.It provides a simple and unified logging interface for your libraries, while making it possible to define the output channel and verbosity level later on.
This way, libraries can be used interchangable between different frameworks, i.e. ROS and RTDB.

## Installation

The  _Generic Logger_ is implemented as a header-only library. So just define the dependency in your package, include the headers and GO!

## Dependencies

No hard dependencies. However, if you want to use the ROS sink, you need to have ROS installed.
This dependency is not listed in the package.xml, but implicitly assumed to be there, when the ros_sink is used.

## Usage
The following MACROS can be used for logging:
* DEBUG_STREAM(...);
* INFO_STREAM(...);
* WARN_STREAM(...);
* ERROR_STREAM(...);

Here are example usages for a library and an executable.
```cpp
// library.h
#include "generic_logger/generic_logger.hpp"
#include ...

int SOME_RANDOM_NUMBER = 42;

void foo(){
    // Possible ways to log, streaming operator can be used as well.
    DEBUG_STREAM("foo function called");
    INFO_STREAM("Wow, look at this number: " << SOME_RANDOM_NUMBER);
    WARN_STREAM("Warning, this function does not do anything useful!");
    ERROR_STREAM("Error: foo");
}
```
**WARNING**:
 * By default the  _Generic Logger_ will log to the **null_sink**. At the beginning of your main() you have to set your preferred sink! 
 * When using the ros_sink you should always set the logger level to **debug**, as you will manage the actuall output through ROS (`rosrun rqt_logger_level rqt_logger_level`)

```cpp
// tool.cpp
#include "library.h"
#include <ros/ros.h>
#include "generic_logger/generic_logger.hpp"
#include "generic_logger/sinks/ros_sink.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ExampleNode");
    ros::NodeHandle node_handle;
    
    // create and set sink
    auto ros_sink = std::make_shared<generic_logger::sinks::ros_sink>();
    generic_logger::set_sink(ros_sink);
    // set verbosity level
    generic_logger::set_level("debug");
    // use logger
    INFO_STREAM("Example Node started");
    
    //...

    return 0;
};
```

## Implementation details
The  _Generic Logger_ is implemented as a singleton. This means, a tool and all libraries used by this tool, share the same logger. When the sink or verbosity level is changed in a tool, it will affect **all** of the used libraries.

By default the  _Generic Logger_ will log to the **null_sink**. At the beginning of your main() you have to set your preferred sink!

The _Generic Logger_ builds upon the [spdlog] library, which is known for being super fast and lightweight.

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

[spdlog]: https://github.com/gabime/spdlog