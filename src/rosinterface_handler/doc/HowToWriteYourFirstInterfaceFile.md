# How to Write Your First .rosif File
**Description**: This tutorial will familiarize you with .rosif files that allow you to use rosinterface_handler.

**Tutorial Level**: INTERMEDIATE

## Basic Setup

To begin lets create a package called rosinterface_tutorials which depends on rospy, roscpp, rosinterface_handler and dynamic_reconfigure.
```shell
catkin_create_pkg --rosdistro ROSDISTRO rospy roscpp rosinterface_handler dynamic_reconfigure
```
Where ROSDISTRO is the ROS distribution you are using.

Now in your package create a cfg directory, this is where all cfg and rosif files live:
```shell
mkdir cfg
```
Lastly you will need to create a rosif file, for this example we'll call it Tutorials.rosif, and open it in your favorite editor.

## The .rosif File

Add the following to your Tutorials.rosif file:
```python
#!/usr/bin/env python
from rosinterface_handler.interface_generator_catkin import *
gen = InterfaceGenerator()

# Parameters with different types
gen.add("int_param", paramtype="int", description="An Integer parameter")
gen.add("double_param", paramtype="double",description="A double parameter")
gen.add("str_param", paramtype="std::string", description="A string parameter",  default="Hello World")
gen.add("bool_param", paramtype="bool", description="A Boolean parameter")
gen.add("vector_param", paramtype="std::vector<double>", description="A vector parameter")
gen.add("map_param", paramtype="std::map<std::string,std::string>", description="A map parameter")

# Default min and max values
gen.add("weight", paramtype="double",description="Weight can not be negative", min=0.0)
gen.add("age", paramtype="int",description="Normal age of a human is inbetween 0 and 100", min=0, max=100)
gen.add("default_param", paramtype="std::string",description="Parameter with default value", default="Hello World")

# Default vector/map
gen.add("vector_bool", paramtype="std::vector<bool>", description="A vector of boolean with default value.", default=[False, True, True, False, True])
gen.add("map_string_float", paramtype="std::map<std::string,float>", description="A map of <std::string,float> with default value.", default={"a":0.1, "b":1.2, "c":2.3, "d":3.4, "e":4.5}, min=0, max=5)

# Constant and configurable parameters
gen.add("optimal_parameter", paramtype="double", description="Optimal parameter, can not be set via rosinterface", default=10, constant=True)
gen.add("configurable_parameter", paramtype="double", description="This parameter can be set via dynamic_reconfigure", configurable=True)

# Defining the namespace
gen.add("global_parameter", paramtype="std::string", description="This parameter is defined in the global namespace", global_scope=True)

# Full signature
gen.add("dummy", paramtype="double", description="My Dummy parameter", level=0,
edit_method="", default=5.2, min=0, max=10, configurable=True,
global_scope=False, constant=False)

# Add an enum:
gen.add_enum("my_enum", description="My first self written enum",
entry_strings=["Small", "Medium", "Large", "ExtraLarge"], default="Medium")

# Add a subgroup
my_group = gen.add_group("my_group")
my_group.add("subparam", paramtype="std::string", description="This parameter is part of a group", configurable=True)

# Create subscribers/publishers
gen.add_subscriber("my_subscriber", message_type="std_msgs::Header", description="subscriber", configurable=True)
gen.add_publisher("my_publisher", message_type="std_msgs::Header", description="publisher", default_topic="publisher_topic")

# Logging
gen.add_verbosity_param(configurable=True)

# TF
gen.add_tf(buffer_name="tf_buffer", listener_name="tf_listener", broadcaster_name="tf_broadcaster")

#Syntax : Package, Node, Config Name(The final name will be MyDummyConfig)
exit(gen.generate("rosinterface_tutorials", "example_node", "Tutorial"))
```

## Line by line
Now lets break the code down line by line.

### Initialization
```python
#!/usr/bin/env python
from rosinterface_handler.interface_generator_catkin import *
gen = InterfaceGenerator()
```
These first lines are pretty simple, they just initialize ros, import and instantiate the parameter generator.

### Adding parameters
Now that we have a generator we can start to define parameters. The add function adds a parameter to the list of parameters. It takes a the following mandatory arguments:

- **name**: a string which specifies the name under which this parameter should be stored
- **paramtype**: defines the type of value stored, and can be any of the primitive types: "int", "double", "std::string", "bool" or a container type using one of the primitive types: "std::vector<...>", "std::map<std::string, ...>"
- **description**: string which describes the parameter

Furthermore, following optional arguments can be passed:
- **configurable**: Make this parameter reconfigurable at run time. Default: False
- **global_scope**: Make this parameter live in the global namespace. Default: False
- **level**: A bitmask which will later be passed to the dynamic reconfigure callback. When the callback is called all of the level values for parameters that have been changed are ORed together and the resulting value is passed to the callback. This is only used when *configurable* is set to True.
- **edit_method**: An optional string that is directly passed to dynamic reconfigure. This is only used when *configurable* is set to True.
- **default**: specifies the default value. Can not be set for global parameters.
- **min**: specifies the min value (optional and does not apply to strings and bools)
- **max**: specifies the max value (optional and does not apply to strings and bools)

```python
# Parameters with different types
gen.add("int_param", paramtype="int", description="An Integer parameter")
gen.add("double_param", paramtype="double",description="A double parameter")
gen.add("str_param", paramtype="std::string", description="A string parameter",  "Hello World")
gen.add("bool_param", paramtype="bool", description="A Boolean parameter")
gen.add("vector_param", paramtype="std::vector<double>", description="A vector parameter")
gen.add("map_param", paramtype="std::map<std::string,std::string>", description="A map parameter")
```

These lines simply define parameters of the different types. Their values will be retrieved from the ros parameter server.

```python
# Default min and max values
gen.add("weight", paramtype="double",description="Weight can not be negative", min=0.0)
gen.add("age", paramtype="int",description="Normal age of a human is inbetween 0 and 100", min=0, max=100)
gen.add("default_param", paramtype="std::string",description="Parameter with default value", default="Hello World")
```

These lines add parameters, which either have default values, which means that they wont throw an error if the parameters are not set on the server, or they have constraints on min/max values for the retrieved parameters. These bounds will be enforced, when fetching parameters from the server.

```python
# Constant and configurable parameters
gen.add("optimal_parameter", paramtype="double", description="Optimal parameter, can not be set via rosinterface", default=10, constant=True)
gen.add("configurable_parameter", paramtype="double", description="This parameter can be set via dynamic_reconfigure", configurable_parameter=True)
```

These lines define a parameter that is configurable and one that is constant. Configurable means, that the entry will be added to the dynamic_reconfigure config file and can later be changed at runtime. Vectors, maps and global parameters can not be set configurable.
A constant parameter can not be set through the parameter server anymore. This can be useful, if you have found optimal parameters for your setup, which should not be changed by users anymore.

```python
# Defining the namespace
gen.add("global_parameter", paramtype="std::string", description="This parameter is defined in the global namespace", global_scope=True)
```

With the global_scope flag, parameters can be defined to live in the global namespace. Normally you should always keep your parameters in the private namespace of a node. Only exception is when several nodes need to get the exact same value for a parameter. (E.g. a common map file)

### Adding enums
```python
# Add an enum:
gen.add_enum("my_enum", description="My first self written enum",
entry_strings=["Small", "Medium", "Large", "ExtraLarge"], default="Medium"))
```

By using the add_enum function, an enum for the dynamic_reconfigure window can be easily defined. The entry_strings will also be static parameters of the resulting parameter struct.

### Creating groups
```python
# Add a subgroup
my_group = gen.add_group("my_group")
my_group.add("subparam", paramtype="std::string", description="This parameter is part of a group", configurable=True)
```

With the add_group function, you can sort parameters into groups in the dynamic_reconfigure window. This obviously only makes sense for configurable parameters.

### Adding publishers/subscribers
```python
# Create subscribers/publishers
gen.add_subscriber("my_subscriber", message_type="std_msgs::Header", description="subscriber", configurable=True)
gen.add_publisher("my_publisher", message_type="std_msgs::Header", description="publisher", default_topic="publisher_topic")
```

Using these commands, the rosinterface handler can even do the subscribing and advertising for you. The rosinterface handler makes sure the node is always connected to the right topic. A parameter for the topic and the queue size will be automatically generated for you.
The signature for both commands are very similar. They take the following mandatory elements:
- **name**: Base name of the subscriber/publisher. Will be name of the object in the parameter struct. The topic parameter is then *name*_topic and the queue size *name*_queue_size (unless overriden).
- **message_type**: Type of message including its namespace (e.g. std_msgs::Header). This will also be used to generate the name of the header/module to include (unless overriden).
- **description**: Chose an informative documentation string for this subscriber/publisher.

The following parameters are optional. Many of them will be automatically deduced from the mandatory parameters:
- **default_topic**: Default topic to subscribe to. If this is an empty string, the subscriber/publisher will initialized in an invalid state. If it is `None` (default), the node will report an error if the topic is not defined by a parameter.
- **default_queue_size**: Default queue size of the subscriber/publisher.
- **no_delay** _(only for add_subscriber)_: Set the tcp_no_delay parameter for subscribing. Recommended for larger messages.
- **topic_param**: Name of the param configuring the topic. Will be "*name*_topic" if None.
- **queue_size_param**: Name of param configuring the queue size. Defaults to "*name*_queue_size".
- **header**: Header name to include. Will be deduced for message type if None.
- **module**: Module to import from (e.g. std_msgs.msg). Will be automatically deduced if None.
- **configurable**: Should the subscribed topic and message queue size be dynamically configurable?
- **scope**: Either *global*, *public* or *private*. A *global* subscriber/publisher will subscribe to `/topic`,
        a *public* one to `/namespace/topic` and a *private* one to `/namespace/node_name/topic`. If the topic starts with "/",
        the node will always subscribe globally.
- **constant**: If this is true, the parameters will not be fetched from param server,
        but the default value is kept.
- **diagnosed**: Enables diagnostics for the subscriber/publisher. Can be configured with the params below.
        The message must have a header. Not yet supported for python.
- **min_frequency**: Sets the default minimum frequency for the subscriber/publisher
- **min_frequency_param**: Sets the parameter for the minimum frequency. Defaults to <name>_min_frequency
- **max_delay**: Sets the default maximal header delay for the topics in seconds.
- **max_delay_param**: Parameter for the maximal delay. Defaults to <name>_max_delay.

To define the topic, just set the topic parameter (usually <my_subscriber>_topic) to the topic of your dreams in your launch or config file.


### Defining verbosity
```python
# Logging
gen.add_verbosity_param(configurable=True)
```

Changing the verbosity of nodes to debug (or silence) something can be annoying.
By calling `add_verbosity_param()`, *rosinterface_handler* adds a verbosity parameter (named *verbosity* by default) for you and will automatically set
the verbosity of the node. If you make the parameter *configurable*, you can comfortably select the verbosity in the
*rqt_reconfigure* window. The following parameters are optional:
- **name**: Name of the verbosity parameter.
- **configurable**: Show the verbosity in the *rqt_reconfigure* window.
- **default**: Initial verbosity (can be `debug`, `info`, `warning`, `error` or `fatal`).
### TF
```python
gen.add_tf(buffer_name="tf_buffer", listener_name="tf_listener", broadcaster_name="tf_broadcaster")
```

The rosinterface handler can also generate the tf objects for you (your package must depend on tf2_ros, of course).
In the end, your interface object will have three more members: A tf_buffer, a tf_listener and a tf_broadcaster that you can use for handling transformations. These are the supported parameters:
- **buffer_name**: Optional: Name of the tf2_ros::Buffer member in the interface object. Required if listener_name is not `None`.
- **listener_name**: Optional: Name of the tf2_ros::TransformListener member in the interface object. Will not be created if `None`
- **broadcaster_name**: Optional: Name of the tf2_ros::TransformBroadcaster member in the interface object. Will not be created if `None`

### Diagnosed publishers
Diagnosed publisher/subscriber are created by passing `diagnosed=True` to the add_subscriber/publisher definition in the interface file.
Before you do this, you must add a line `gen.add_diagnostic_updater()` to your file and not forget to add _diagnostic_updater_ as a dependency to your package.
You can control the expected minimal frequency by setting the respective parameter. The delay of the messages can be monitored like this as well.
 
Currently this is not supported for python (the flag is ignored).



### The final step

```python
exit(gen.generate("rosinterface_tutorials", "example_node", "Tutorial"))
```

The last line simply tells the generator to generate the necessary files and exit the program. The second parameter is the name of a node this could run in (used to generate documentation only), the third parameter is a name prefix the generated files will get (e.g. "<name>Config.h" for the dynamic_reconfigure struct and "<name>Parameters.h" for the parameter struct.

NOTE: The third parameter should be equal to the .rosif file name, without extension. Otherwise the libraries will be generated in every build, forcing a recompilation of the nodes which use them.

## Add rosif file to CMakeLists

In order to make this rosif file usable it must be executable, so lets use the following command to make it excecutable

```shell
chmod a+x cfg/Tutorials.rosif
```

Next we need to add the following lines to our CMakeLists.txt.

```cmake
#add rosinterface_handler api
find_package(catkin REQUIRED rosinterface_handler dynamic_reconfigure)

generate_ros_interface_files(
  cfg/Tutorials.rosif
  cfg/SomeOtherCfgFile.cfg
  #...
)

# make sure configure headers are built before any node using them
add_dependencies(example_node ${PROJECT_NAME}_gencfg) # For dynamic_reconfigure
add_dependencies(example_node ${PROJECT_NAME}_geninterface) # For rosinterface_handler
```
Note: You need a node example_node that is already built, before the add_dependencies line is reached (ref Create a node in C++).  

This will run our rosif file when the package is built. The last thing to do is to build the package and we're done!

Note: It should be noted here, that you have to pass **all** '.rosif' **and all** '.cfg' files to the generate_parameter_files call. This is because dynamic_reconfigure can only be called once per package. Your normal cfg files will be passed on together with the newly created cfg files.

For information on how to use the resulting parameter struct in your code, see the next tutorial on [How to use your interface struct](HowToUseYourInterfaceStruct.md).
