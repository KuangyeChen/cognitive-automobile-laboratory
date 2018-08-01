# Copyright (c) 2016, Claudio Bandera
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the organization nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Claudio Bandera
#

from __future__ import print_function
from string import Template
import sys
import os
import re

def eprint(*args, **kwargs):
    print("************************************************", file=sys.stderr, **kwargs)
    print("Error when setting up parameter '{}':".format(args[0]), file=sys.stderr, **kwargs)
    print(*args[1:], file=sys.stderr, **kwargs)
    print("************************************************", file=sys.stderr, **kwargs)
    sys.exit(1)


class InterfaceGenerator(object):
    """Automatic config file and header generator"""

    def __init__(self, parent=None, group=""):
        """Constructor for InterfaceGenerator"""
        self.enums = []
        self.parameters = []
        self.subscribers = []
        self.publishers = []
        self.childs = []
        self.tf = None
        self.verbosity = None
        self.parent = parent
        self.diagnostics_enabled = False
        if group:
            self.group = group
        else:
            self.group = "gen"
        self.group_variable = filter(str.isalnum, self.group)

        if len(sys.argv) != 5:
            eprint(
                "InterfaceGenerator: Unexpected amount of args, did you try to call this directly? You shouldn't do this!")

        self.dynconfpath = sys.argv[1]
        self.share_dir = sys.argv[2]
        self.cpp_gen_dir = sys.argv[3]
        self.py_gen_dir = sys.argv[4]

        self.pkgname = None
        self.nodename = None
        self.classname = None

    def add_verbosity_param(self, name='verbosity', default='info', configurable=False):
        """
        Adds a parameter that will define the logging verbosity of the node. The verbosity can either be 'debug',
        'info', 'warning', 'error' or 'fatal'.
        The verbosity will be updated automatically by fromParamServer() or fromConfig().
        :param name: (optional) Name of the parameter that will define the verbosity
        :param default: (optional) Default verbosity
        :param configurable: (optional) Should the topic name and message queue size be dynamically configurable?
        :return:
        """
        if self.verbosity:
            eprint("add_verbosity_param has already been called. You can only add one verbosity parameter per file!")
        if self.parent:
            eprint("You can not call add_verbosity_param on a group! Call it on the main parameter generator instead!")
        self.verbosity = name
        if configurable:
            self.add_enum(name, 'Sets the verbosity for this node',
                          entry_strings=['debug', 'info', 'warning', 'error', 'fatal'],
                          default=default,
                          paramtype='std::string')
        else:
            self.add(name, description='Sets the verbosity for this node', paramtype='std::string', default=default)

    def add_diagnostic_updater(self):
        """
        Adds a diagnostic updater to the interface struct. Make sure your project depends on the diagnostic_updater
        package. Must be called before adding any diagnostic-enabled publishers/subscribers
        :return:
        """
        if self.parent:
            eprint("You can not call add_diagnostic_updater on a group! Call it on the main parameter generator instead!")
        self.diagnostics_enabled = True

    def add_tf(self, buffer_name="tf_buffer", listener_name="tf_listener", broadcaster_name=None):
        """
        Adds tf transformer/broadcaster as members to the interface object. Don't forget to depend your package on tf2_ros.
        :param buffer_name: Name of the tf2_ros::Buffer member in the interface object
        :param listener_name: Name of the tf2_ros::TransformListener member in the interface object.
        Will not be created if none.
        :param broadcaster_name: Name of the tf2_ros::TransformBroadcaster member in the interface object.
        Will not be created if none.
        """
        if self.parent:
            eprint("You can not call add_tf on a group! Call it on the main parameter generator instead!")
        if listener_name and not buffer_name:
            eprint("If you specify a tf listener, the tf buffer can not be empty!")
        self.tf = {"buffer_name": buffer_name, "listener_name": listener_name, "broadcaster_name": broadcaster_name}


    def add_group(self, name):
        """
        Add a new group in the dynamic reconfigure selection
        :param name: name of the new group
        :return: a new group object that you can add parameters to
        """
        if not name:
            eprint("You have added a group with an empty group name. This is not supported!")
        child = InterfaceGenerator(self, name)
        self.childs.append(child)
        return child

    def add_enum(self, name, description, entry_strings, default=None, paramtype='int'):
        """
        Add an enum to dynamic reconfigure
        :param name: Name of enum parameter
        :param description: Informative documentation string
        :param entry_strings: Enum entries, must be strings! (will be numbered with increasing value)
        :param default: Default value
        :param paramtype: 'int' or 'std::string'; defines if selecting an entry will set the parameter to the name of
        an entry or to an int (enumerated from zero in the order of the list)
        :return:
        """
        if paramtype != 'int' and paramtype != 'std::string':
            eprint("Only 'int' or 'std::string' is supported as paramtype argument for add_enum!")

        entry_strings = [str(e) for e in entry_strings]  # Make sure we only get strings
        if default is None:
            default = 0
        elif paramtype == 'int':
            default = entry_strings.index(default)
        elif not default in entry_strings:
            eprint("add_enum: Default value not found in entry_strings!")

        self.add(name=name, paramtype=paramtype, description=description, edit_method=name, default=default,
                 configurable=True)
        for e in entry_strings:
            enum_val = entry_strings.index(e) if paramtype == 'int' else e
            self.add(name=name + "_" + e, paramtype=paramtype, description="Constant for enum {}".format(name),
                     default=enum_val, constant=True)
        self.enums.append({'name': name, 'description': description, 'values': entry_strings, 'paramtype': paramtype})

    def add_subscriber(self, name, message_type, description, default_topic=None, default_queue_size=5, no_delay=False,
                       topic_param=None, queue_size_param=None, header=None, module=None, configurable=False,
                       scope='private', constant=False, diagnosed=False, min_frequency=0., min_frequency_param=None,
                       max_delay=float('inf'), max_delay_param=None):
        """
        Adds a subscriber to your parameter struct and a parameter for its topic and queue size. Don't forget to add a
        dependency to message_filter and the package for the message used to your package.xml!
        :param name: Base name of the subscriber. Will be name of the subscriber in the parameter struct. The topic
        parameter is then <name>_topic and the queue size <name>_queue_size (unless overriden).
        :param message_type: Type of message including its namespace (e.g. std_msgs::Header)
        :param description: Chose an informative documentation string for this subscriber.
        :param default_topic: (optional) Default topic to subscribe to.
        :param default_queue_size: (optional) Default queue size of the subscriber.
        :param no_delay: (optional) Set the tcp_no_delay parameter for subscribing. Recommended for larger topics.
        :param topic_param: (optional) Name of the param configuring the topic. Will be "<name>_topic" if None.
        :param queue_size_param: (optional) Name of param configuring the queue size. Defaults to "<name>_queue_size".
        :param header: (optional) Header name to include. Will be deduced for message type if None.
        :param module: (optional) Module to import from (e.g. std_msgs.msg). Will be automatically deduced if None.
        :param configurable: (optional) Should the topic name and message queue size be dynamically configurable?
        :param scope: (optional) Either "global", "public" or "private". A "global" subscriber will subscribe to /topic,
        a "public" one to /namespace/topic and a "private" one to /namespace/node_name/topic.
        :param constant: (optional) If this is true, the parameters will not be fetched from param server,
        but the default value is kept.
        :param diagnosed: (optional) Enables diagnostics for the subscriber. Can be configured with the params below.
        The message must have a header. Not yet supported for python.
        :param min_frequency: (optional) Sets the default minimum frequency for the subscriber
        :param min_frequency_param: (optional) Sets the parameter for the minimum frequency. Defaults to <name>_min_frequency
        :param max_delay: (optional) Sets the default maximal header delay for the topics in seconds.
        :param max_delay_param: (optional) Parameter for the maximal delay. Defaults to <name>_max_delay.
        :return: None
        """
        # add subscriber topic and queue size as param
        if not topic_param:
            topic_param = name + '_topic'
        if not queue_size_param:
            queue_size_param = name + '_queue_size'
        self.add(name=topic_param, paramtype='std::string', description='Topic for ' + description,
                 default=default_topic, configurable=configurable, global_scope=False, constant=constant)
        self.add(name=queue_size_param, paramtype='int', description='Queue size for ' + description, min=0,
                 default=default_queue_size, configurable=configurable, global_scope=False, constant=constant)

        if diagnosed:
            if not self._get_root().diagnostics_enabled:
                eprint("Please enable diagnostics before adding a diagnosed subscriber")
            if not min_frequency_param:
                min_frequency_param = name + '_min_frequency'
            if not max_delay_param:
                max_delay_param = name + '_max_delay'
            self.add(name=min_frequency_param, paramtype='double', description='Minimal message frequency for ' + description,
                     default=min_frequency, configurable=configurable, global_scope=False, constant=constant)
            self.add(name=max_delay_param, paramtype='double', description='Maximal delay for ' + description,
                     default=max_delay, configurable=configurable, global_scope=False, constant=constant)

        # normalize the topic type (we want it to contain ::)
        normalized_message_type = message_type.replace("/", "::").replace(".", "::")

        if not module:
            module = ".".join(normalized_message_type.split('::')[0:-1]) + '.msg'

        # add a subscriber object
        newparam = {
            'name': name,
            'type': normalized_message_type,
            'header': header,
            'import': module,
            'topic_param': topic_param,
            'queue_size_param': queue_size_param,
            'no_delay': no_delay,
            'configurable': configurable,
            'description': description,
            'scope': scope.lower(),
            'diagnosed': diagnosed,
            'min_frequency_param': min_frequency_param,
            'max_delay_param': max_delay_param
        }
        self.subscribers.append(newparam)

    def add_publisher(self, name, message_type, description, default_topic=None, default_queue_size=5, topic_param=None,
                      queue_size_param=None, header=None, module=None, configurable=False, scope="private",
                      constant=False, diagnosed=False, min_frequency=0., min_frequency_param=None, max_delay=float('inf'),
                      max_delay_param=None):
        """
        Adds a publisher to your parameter struct and a parameter for its topic and queue size. Don't forget to add a
        dependency to message_filter and the package for the message used to your package.xml!
        :param name: Base name of the publisher. Will be name of the publisher in the parameter struct. The topic
        parameter is then <name>_topic and the queue size <name>_queue_size (unless overriden).
        :param message_type: Type of message including its namespace (e.g. std_msgs::Header)
        :param description: Chose an informative documentation string for this publisher.
        :param default_topic: (optional) Default topic to publish to.
        :param default_queue_size: (optional) Default queue size of the publisher.
        :param topic_param: (optional) Name of the param configuring the topic. Will be "<name>_topic" if None.
        :param queue_size_param: (optional) Name of param configuring the queue size. Defaults to "<name>_queue_size".
        :param header: (optional) Header name to include. Will be deduced for message type if None.
        :param module: (optional) Module to import from (e.g. std_msgs.msg). Will be automatically deduced if None.
        :param configurable: (optional) Should the topic name and message queue size be dynamically configurable?
        :param scope: (optional) Either "global", "public" or "private". A "global" subscriber will subscribe to /topic,
        a "public" one to /namespace/topic and a "private" one to /namespace/node_name/topic.
        :param constant: (optional) If this is true, the parameters will not be fetched from param server,
        but the default value is kept.
        :param diagnosed: (optional) Enables diagnostics for the publisher. Can be configured with the params below.
        The message must have a header. Not yet supported for python.
        :param min_frequency: (optional) Sets the default minimum frequency for the publisher
        :param min_frequency_param: (optional) Sets the parameter for the minimum frequency. Defaults to <name>_min_frequency
        :param max_delay: (optional) Sets the default maximal header delay for the topics in seconds.
        :param max_delay_param: (optional) Parameter for the maximal delay. Defaults to <name>_max_delay.
        :return: None
        """
        # add publisher topic and queue size as param
        if not topic_param:
            topic_param = name + '_topic'
        if not queue_size_param:
            queue_size_param = name + '_queue_size'
        self.add(name=topic_param, paramtype='std::string', description='Topic for ' + description,
                 default=default_topic, configurable=configurable, global_scope=False, constant=constant)
        self.add(name=queue_size_param, paramtype='int', description='Queue size for ' + description, min=0,
                 default=default_queue_size, configurable=configurable, global_scope=False, constant=constant)

        if diagnosed:
            if not self._get_root().diagnostics_enabled:
                eprint("Please enable diagnostics before adding a diagnosed publisher")
            if not min_frequency_param:
                min_frequency_param = name + '_min_frequency'
            if not max_delay_param:
                max_delay_param = name + '_max_delay'
            self.add(name=min_frequency_param, paramtype='double', description='Minimal message frequency for ' + description,
                     default=min_frequency, configurable=configurable, global_scope=False, constant=constant)
            self.add(name=max_delay_param, paramtype='double', description='Maximal delay for ' + description,
                     default=max_delay, configurable=configurable, global_scope=False, constant=constant)

        # normalize the topic type (we want it to contain ::)
        normalized_message_type = message_type.replace("/", "::").replace(".", "::")

        if not module:
            module = ".".join(normalized_message_type.split('::')[0:-1]) + '.msg'

        # add a publisher object
        newparam = {
            'name': name,
            'type': normalized_message_type,
            'header': header,
            'import': module,
            'topic_param': topic_param,
            'queue_size_param': queue_size_param,
            'configurable': configurable,
            'description': description,
            'scope': scope.lower(),
            'diagnosed': diagnosed,
            'min_frequency_param': min_frequency_param,
            'max_delay_param': max_delay_param
        }
        self.publishers.append(newparam)


    def add(self, name, paramtype, description, level=0, edit_method='""', default=None, min=None, max=None,
            configurable=False, global_scope=False, constant=False):
        """
        Add parameters to your parameter struct. Call this method from your .params file!

        - If no default value is given, you need to specify one in your launch file
        - Global parameters, vectors, maps and constant params can not be configurable
        - Global parameters, vectors and maps can not have a default, min or max value

        :param self:
        :param name: The Name of you new parameter
        :param paramtype: The C++ type of this parameter. Can be any of ['std::string', 'int', 'bool', 'float',
        'double'] or std::vector<...> or std::map<std::string, ...>
        :param description: Choose an informative documentation string for this parameter.
        :param level: (optional) Passed to dynamic_reconfigure
        :param edit_method: (optional) Passed to dynamic_reconfigure
        :param default: (optional) default value
        :param min: (optional)
        :param max: (optional)
        :param configurable: (optional) Should this parameter be dynamic configurable
        :param global_scope: (optional) If true, parameter is searched in global ('/') namespace instead of private (
        '~') ns
        :param constant: (optional) If this is true, the parameter will not be fetched from param server,
        but the default value is kept.
        :return: None
        """
        configurable = self._make_bool(configurable)
        global_scope = self._make_bool(global_scope)
        constant = self._make_bool(constant)
        newparam = {
            'name': name,
            'type': paramtype,
            'default': default,
            'level': level,
            'edit_method': edit_method,
            'description': description,
            'min': min,
            'max': max,
            'is_vector': False,
            'is_map': False,
            'configurable': configurable,
            'constant': constant,
            'global_scope': global_scope,
        }
        self._perform_checks(newparam)
        self.parameters.append(newparam)

    def _perform_checks(self, param):
        """
        Will test some logical constraints as well as correct types.
        Throws Exception in case of error.
        :param self:
        :param param: Dictionary of one param
        :return:
        """

        in_type = param['type'].strip()
        if param['max'] is not None or param['min'] is not None:
            if in_type in ["std::string", "bool"]:
                eprint(param['name'], "Max and min can not be specified for variable of type %s" % in_type)

        if in_type.startswith('std::vector'):
            param['is_vector'] = True
        if in_type.startswith('std::map'):
            param['is_map'] = True

        if (param['is_vector']):
            if (param['max'] is not None or param['min'] is not None):
                ptype = in_type[12:-1].strip()
                if ptype == "std::string":
                    eprint(param['name'], "Max and min can not be specified for variable of type %s" % in_type)

        if (param['is_map']):
            if (param['max'] is not None or param['min'] is not None):
                ptype = in_type[9:-1].split(',')
                if len(ptype) != 2:
                    eprint(param['name'],
                           "Wrong syntax used for setting up std::map<... , ...>: You provided '%s' with "
                           "parameter %s" % in_type)
                ptype = ptype[1].strip()
                if ptype == "std::string":
                    eprint(param['name'], "Max and min can not be specified for variable of type %s" % in_type)

        pattern = r'^[a-zA-Z][a-zA-Z0-9_]*$'
        if not re.match(pattern, param['name']):
            eprint(param['name'], "The name of field does not follow the ROS naming conventions, "
                                  "see http://wiki.ros.org/ROS/Patterns/Conventions")
        if param['configurable'] and (
                            param['global_scope'] or param['is_vector'] or param['is_map'] or param['constant']):
            eprint(param['name'],
                   "Global parameters, vectors, maps and constant params can not be declared configurable! ")
        if param['global_scope'] and param['default'] is not None:
            eprint(param['name'], "Default values for global parameters should not be specified in node! ")
        if param['constant'] and param['default'] is None:
            eprint(param['name'], "Constant parameters need a default value!")
        if param['name'] in [p['name'] for p in self.parameters]:
            eprint(param['name'], "Parameter with the same name exists already")
        if param['edit_method'] == '':
            param['edit_method'] = '""'
        elif param['edit_method'] != '""':
            param['configurable'] = True

        # Check type
        if param['is_vector']:
            ptype = in_type[12:-1].strip()
            self._test_primitive_type(param['name'], ptype)
            param['type'] = 'std::vector<{}>'.format(ptype)
        elif param['is_map']:
            ptype = in_type[9:-1].split(',')
            if len(ptype) != 2:
                eprint(param['name'], "Wrong syntax used for setting up std::map<... , ...>: You provided '%s' with "
                                      "parameter %s" % in_type)
            ptype[0] = ptype[0].strip()
            ptype[1] = ptype[1].strip()
            if ptype[0] != "std::string":
                eprint(param['name'], "Can not setup map with %s as key type. Only std::map<std::string, "
                                      "...> are allowed" % ptype[0])
            self._test_primitive_type(param['name'], ptype[0])
            self._test_primitive_type(param['name'], ptype[1])
            param['type'] = 'std::map<{},{}>'.format(ptype[0], ptype[1])
        else:
            # Pytype and defaults can only be applied to primitives
            self._test_primitive_type(param['name'], in_type)
            param['pytype'] = self._pytype(in_type)

    @staticmethod
    def _pytype(drtype):
        """Convert C++ type to python type"""
        return {'std::string': "str", 'int': "int", 'double': "double", 'bool': "bool"}[drtype]

    @staticmethod
    def _test_primitive_type(name, drtype):
        """
        Test whether parameter has one of the accepted C++ types
        :param name: Interfacename
        :param drtype: Typestring
        :return:
        """
        primitive_types = ['std::string', 'int', 'bool', 'float', 'double']
        if drtype not in primitive_types:
            raise TypeError("'%s' has type %s, but allowed are: %s" % (name, drtype, primitive_types))

    @staticmethod
    def _get_cvalue(param, field):
        """
        Helper function to convert strings and booleans to correct C++ syntax
        :param param:
        :return: C++ compatible representation
        """
        value = param[field]
        if param['type'] == 'double' and param[field] == float('inf'):
            value = 'std::numeric_limits<double>::infinity()'
        if param['type'] == 'double' and param[field] == float('-inf'):
            value = '-std::numeric_limits<double>::infinity()'
        if param['type'] == 'std::string':
            value = '"{}"'.format(param[field])
        elif param['type'] == 'bool':
            value = str(param[field]).lower()
        return str(value)

    @staticmethod
    def _get_pyvalue(param, field):
        """
        Helper function to convert strings and booleans to correct C++ syntax
        :param param:
        :return: C++ compatible representation
        """
        value = param[field]
        if param['type'] == 'double' and param[field] == float('inf'):
            value = '1e+308' # dynamic reconfigure does not supprt if, but this should be sufficient.
        if param['type'] == 'double' and param[field] == float('-inf'):
            value = '-1e+308'
        if param['type'] == 'std::string':
            value = '"{}"'.format(param[field])
        elif param['type'] == 'bool':
            value = str(param[field]).capitalize()
        return str(value)

    @staticmethod
    def _get_cvaluelist(param, field):
        """
        Helper function to convert python list of strings and booleans to correct C++ syntax
        :param param:
        :return: C++ compatible representation
        """
        values = param[field]
        assert (isinstance(values, list))
        form = ""
        for value in values:
            if param['type'] == 'std::vector<std::string>':
                value = '"{}"'.format(value)
            elif param['type'] == 'std::vector<bool>':
                value = str(value).lower()
            else:
                value = str(value)
            form += value + ','
        # remove last ','
        return form[:-1]

    @staticmethod
    def _get_cvaluedict(param, field):
        """
        Helper function to convert python dict of strings and booleans to correct C++ syntax
        :param param:
        :return: C++ compatible representation
        """
        values = param[field]
        assert (isinstance(values, dict))
        form = ""
        for key, value in values.items():
            if param['type'] == 'std::map<std::string,std::string>':
                pair = '{{"{}","{}"}}'.format(key, value)
            elif param['type'] == 'std::map<std::string,bool>':
                pair = '{{"{}",{}}}'.format(key, str(value).lower())
            else:
                pair = '{{"{}",{}}}'.format(key, str(value))
            form += pair + ','
        # remove last ','
        return form[:-1]

    def generate(self, pkgname, nodename, classname):
        """
        Main working Function, call this at the end of your .params file!
        :param self:
        :param pkgname: Name of the catkin package
        :param nodename: Name of the Node that will hold these params
        :param classname: This should match your file name, so that cmake will detect changes in config file.
        :return: Exit Code
        """
        self.pkgname = pkgname
        self.nodename = nodename
        self.classname = classname

        print("Generating interface file for node {} (class {}) in package {}".format(nodename, classname, pkgname))

        if self.parent:
            eprint("You should not call generate on a group! Call it on the main parameter generator instead!")

        return self._generateImpl()

    def _generateImpl(self):
        """
        Implementation level function. Can be overwritten by derived classes.
        :return:
        """
        self._generatecfg()
        self._generatehpp()
        self._generatepy()

        return 0

    def _generatecfg(self):
        """
        Generate .cfg file for dynamic reconfigure
        :param self:
        :return:
        """
        templatefile = os.path.join(self.dynconfpath, "templates", "ConfigType.h.template")
        with open(templatefile, 'r') as f:
            template = f.read()

        param_entries = self._generate_param_entries()

        param_entries = "\n".join(param_entries)
        template = Template(template).substitute(pkgname=self.pkgname, nodename=self.nodename,
                                                 classname=self.classname, params=param_entries)

        cfg_file = os.path.join(self.share_dir, "cfg", self.classname + ".cfg")
        try:
            if not os.path.exists(os.path.dirname(cfg_file)):
                os.makedirs(os.path.dirname(cfg_file))
        except OSError:
            # Stupid error, sometimes the directory exists anyway
            pass
        with open(cfg_file, 'w') as f:
            f.write(template)
        os.chmod(cfg_file, 509)  # entspricht 775 (octal)

    def _generatehpp(self):
        """
        Generate C++ Header file, holding the parameter struct.
        :param self:
        :return:
        """

        # Read in template file
        templatefile = os.path.join(self.dynconfpath, "templates", "Interface.h.template")
        with open(templatefile, 'r') as f:
            template = f.read()

        param_entries = []
        string_representation = []
        from_server = []
        to_server = []
        non_default_params = []
        from_config = []
        test_limits = []
        includes = []
        sub_adv_from_server = []
        sub_adv_from_config = []
        subscriber_entries = []
        subscribers_init = []
        publisher_entries = []

        subscribers = self._get_subscribers()
        publishers = self._get_publishers()
        if subscribers or publishers:
            include_error = "#error message_filters was not found during compilation. " \
                            "Please recompile with message_filters."
        else:
            include_error = ""

        if self.diagnostics_enabled:
            param_entries.append('diagnostic_updater::Updater updater;')
            subscribers_init.append(',\n    updater{ros::NodeHandle(), private_node_handle, "/"+nodeName_}')
            includes.append('#include <rosinterface_handler/diagnostic_subscriber.hpp>')
            from_server.append('    updater.setHardwareID("none");')

        if self.tf:
            listener = self.tf["listener_name"]
            buffer = self.tf["buffer_name"]
            broadcaster = self.tf["broadcaster_name"]
            if buffer:
                includes.append('#include <tf2_ros/buffer.h>')
                param_entries.append('tf2_ros::Buffer {};'.format(buffer))
            if listener:
                includes.append('#include <tf2_ros/transform_listener.h>')
                param_entries.append('tf2_ros::TransformListener {};'.format(listener))
                subscribers_init.append(',\n    {}{{{}}}'.format(listener, buffer))
            if broadcaster:
                includes.append('#include <tf2_ros/transform_broadcaster.h>')
                param_entries.append('tf2_ros::TransformBroadcaster {};'.format(broadcaster))


        for subscriber in subscribers:
            name = subscriber['name']
            type = subscriber['type']
            header = subscriber['header']
            description = subscriber['description']
            diagnosed = subscriber['diagnosed']

            # add include entry
            if header:
                include = "#include <{}>".format(header)
            else:
                type_slash = type.replace("::", "/")
                include = "#include <{}.h>".format(type_slash)
            if not include in includes:
                includes.append(include)

            # add subscriber entry
            if diagnosed:
                diag = "Diag"
                init = "updater"
            else:
                diag = ""
                init = ""

            subscriber_entries.append(Template('  ${diag}SubscriberPtr<${type}> ${name}; /*!< $description '
                                               '*/').substitute(diag=diag, type=type,
                                                                name=name, description=description))

            # add initialisation
            subscribers_init.append(Template(',\n    $name{std::make_shared<${diag}Subscriber<$type>>(${init})}')
                                   .substitute(name=name, diag=diag, type=type, init=init))

            # add subscribe for parameter server
            topic_param = subscriber['topic_param']
            queue_size_param = subscriber['queue_size_param']
            min_freq_param = subscriber['min_frequency_param']
            max_delay_param = subscriber['max_delay_param']
            scope = subscriber['scope']
            if scope == 'private':
                name_space = "privateNamespace_"
            elif scope == 'public':
                name_space = "publicNamespace_"
            elif scope == 'global':
                name_space = "globalNamespace_"
            else:
                eprint("Unknown scope specified for subscriber {}: {}".format(name, scope))
            if subscriber['no_delay']:
                no_delay = ", ros::TransportHints().tcpNoDelay()"
            else:
                no_delay = ""
            if diagnosed:
                sub_adv_from_server.append(Template('    $name->minFrequency($minFParam).maxTimeDelay($maxTParam);')
                                           .substitute(name=name, minFParam=min_freq_param, maxTParam=max_delay_param))
            sub_adv_from_server.append(Template('    $name->subscribe(privateNodeHandle_, '
                                                'rosinterface_handler::getTopic($namespace, $topic), uint32_t($queue)$noDelay);')
                                       .substitute(name=name, topic=topic_param,queue=queue_size_param,
                                                   noDelay=no_delay, namespace=name_space))
            if subscriber['configurable']:
                if diagnosed:
                    sub_adv_from_config.append(Template('    $name->minFrequency(config.$minFParam)'
                                                        '.maxTimeDelay(config.$maxTParam);')
                                               .substitute(name=name, minFParam=min_freq_param, maxTParam=max_delay_param))
                sub_adv_from_config.append(Template('    if($topic != config.$topic || $queue != config.$queue) {\n'
                                                    '      $name->subscribe(privateNodeHandle_, '
                                                    'rosinterface_handler::getTopic($namespace, config.$topic), '
                                                    'uint32_t(config.$queue)$noDelay);\n'
                                                    '    }').substitute(name=name,topic=topic_param,
                                                                        queue=queue_size_param, noDelay=no_delay,
                                                                        namespace=name_space))

        for publisher in publishers:
            name = publisher['name']
            type = publisher['type']
            header = publisher['header']
            description = publisher['description']
            diagnosed = publisher['diagnosed']

            # add include entry
            if header:
                include = "#include <{}>".format(header)
            else:
                type_slash = type.replace("::", "/")
                include = "#include <{}.h>".format(type_slash)
            if not include in includes:
                includes.append(include)

            # add publisher entry
            if diagnosed:
                publish = Template("DiagPublisher<$type>").substitute(type=type)
                init = "{updater}"
            else:
                publish = "ros::Publisher"
                init = ""

            publisher_entries.append(Template('  $publisher ${name}$init; /*!< $description */').substitute(
                publisher=publish, name=name, description=description, init=init))

            # add advertise for parameter server
            topic_param = publisher['topic_param']
            queue_size_param = publisher['queue_size_param']
            min_freq_param = publisher['min_frequency_param']
            max_delay_param = publisher['max_delay_param']
            scope = publisher['scope'].lower()
            if scope == 'private':
                name_space = "privateNamespace_"
            elif scope == 'public':
                name_space = "publicNamespace_"
            elif scope == 'global':
                name_space = "globalNamespace_"
            else:
                eprint("Unknown scope specified for publisher {}: {}".format(name, scope))
            if diagnosed:
                sub_adv_from_server.append(Template('    $name.minFrequency($minFParam).maxTimeDelay($maxTParam);')
                                           .substitute(name=name, minFParam=min_freq_param, maxTParam=max_delay_param))
            sub_adv_from_server.append(Template('    $name = privateNodeHandle_.advertise<$type>('
                                                'rosinterface_handler::getTopic($namespace, $topic), $queue);')
                                       .substitute(name=name, type=type, topic=topic_param, queue=queue_size_param,
                                                   namespace=name_space))
            if publisher['configurable']:
                if diagnosed:
                    sub_adv_from_config.append(Template('    $name.minFrequency(config.$minFParam)'
                                                        '.maxTimeDelay(config.$maxTParam);')
                                               .substitute(name=name, minFParam=min_freq_param, maxTParam=max_delay_param))
                sub_adv_from_config.append(Template('    if($topic != config.$topic || $queue != config.$queue) {\n'
                                                    '      $name = privateNodeHandle_.advertise<$type>('
                                                    'rosinterface_handler::getTopic($namespace, config.$topic), '
                                                    'config.$queue);\n'
                                                    '    }').substitute(name=name, type=type, topic=topic_param,
                                                                        queue=queue_size_param,
                                                                        namespace=name_space))
        includes = "\n".join(includes)
        subscriber_entries = "\n".join(subscriber_entries)
        publisher_entries = "\n".join(publisher_entries)
        sub_adv_from_server = "\n".join(sub_adv_from_server)
        sub_adv_from_config = "\n".join(sub_adv_from_config)
        subscribers_init = "".join(subscribers_init)


        params = self._get_parameters()

        # Create dynamic parts of the header file for every parameter
        for param in params:
            name = param['name']

            # Adjust key for parameter server
            if param["global_scope"]:
                namespace = 'globalNamespace_'
            else:
                namespace = 'privateNamespace_'
            full_name = '{} + "{}"'.format(namespace, param["name"])

            # Test for default value
            if param["default"] is None:
                default = ""
                non_default_params.append(Template('      << "\t" << $namespace << "$name" << " ($type) '
                                                   '\\n"\n').substitute(
                    namespace=namespace, name=name, type=param["type"]))
            else:
                if param['is_vector']:
                    default = ', {}'.format(str(param['type']) + "{" + self._get_cvaluelist(param, "default") + "}")
                elif param['is_map']:
                    default = ', {}'.format(str(param['type']) + "{" + self._get_cvaluedict(param, "default") + "}")
                else:
                    default = ', {}'.format(str(param['type']) + "{" + self._get_cvalue(param, "default") + "}")

            # Test for constant value
            if param['constant']:
                param_entries.append(Template('  static constexpr auto ${name} = $default; /*!< ${description} '
                                              '*/').substitute(type=param['type'], name=name,
                                                               description=param['description'],
                                                               default=self._get_cvalue(param, "default")))
                from_server.append(Template('    rosinterface_handler::testConstParam($paramname);').substitute(paramname=full_name))
            else:
                param_entries.append(Template('  ${type} ${name}; /*!< ${description} */').substitute(
                    type=param['type'], name=name, description=param['description']))
                from_server.append(Template('    success &= rosinterface_handler::getParam($paramname, $name$default);').substitute(
                    paramname=full_name, name=name, default=default, description=param['description']))
                to_server.append(
                    Template('    rosinterface_handler::setParam(${paramname},${name});').substitute(paramname=full_name, name=name))

            # Test for configurable params
            if param['configurable']:
                from_config.append(Template('    $name = config.$name;').substitute(name=name))

            # Test limits
            if param['is_vector']:
                ttype = param['type'][12:-1].strip()
            elif param['is_map']:
                ttype = param['type'][9:-1].strip()
            else:
                ttype = param['type']
            if param['min'] is not None:
                test_limits.append(Template('    rosinterface_handler::testMin<$type>($paramname, $name, $min);').substitute(
                    paramname=full_name, name=name, min=param['min'], type=ttype))
            if param['max'] is not None:
                test_limits.append(Template('    rosinterface_handler::testMax<$type>($paramname, $name, $max);').substitute(
                    paramname=full_name, name=name, max=param['max'], type=ttype))

            # Add debug output
            string_representation.append(Template('      << "\t" << p.$namespace << "$name:" << p.$name << '
                                                  '"\\n"\n').substitute(namespace=namespace, name=name))

        # set verbosity (must be last line in from server and first in from config)
        if self.verbosity:
            from_server.append(Template('    rosinterface_handler::setLoggerLevel(privateNodeHandle_, "$verbosity");').substitute(
                verbosity=self.verbosity))
            param_entry = next((entry for entry in params if entry['name'] == self.verbosity), None)
            if param_entry and param_entry['configurable']:
                verb_check = Template('    if(config.$verbosity != this->$verbosity) {\n'
                                      '        rosinterface_handler::setParam(privateNamespace_ + "$verbosity", config.$verbosity);\n'
                                      '        rosinterface_handler::setLoggerLevel(privateNodeHandle_, "$verbosity");\n'
                                      '    }').substitute(verbosity=self.verbosity)
                from_config.insert(0, verb_check)

        param_entries = "\n".join(param_entries)
        string_representation = "".join(string_representation)
        non_default_params = "".join(non_default_params)
        from_server = "\n".join(from_server)
        to_server = "\n".join(to_server)
        from_config = "\n".join(from_config)
        test_limits = "\n".join(test_limits)

        content = Template(template).substitute(pkgname=self.pkgname, ClassName=self.classname,
                                                parameters=param_entries, fromConfig=from_config,
                                                fromParamServer=from_server,
                                                string_representation=string_representation,
                                                non_default_params=non_default_params, nodename=self.nodename,
                                                test_limits=test_limits, toParamServer=to_server,
                                                includes=includes, includeError=include_error,
                                                subscribeAdvertiseFromParamServer=sub_adv_from_server,
                                                subscribeAdvertiseFromConfig=sub_adv_from_config,
                                                subscribers=subscriber_entries, publishers=publisher_entries,
                                                initSubscribers=subscribers_init)

        header_file = os.path.join(self.cpp_gen_dir, self.classname + "Interface.h")
        try:
            if not os.path.exists(os.path.dirname(header_file)):
                os.makedirs(os.path.dirname(header_file))
        except OSError:
            # Stupid error, sometimes the directory exists anyway
            pass
        with open(header_file, 'w') as f:
            f.write(content)

    def _generatepy(self):
        """
        Generate Python parameter file
        :param self:
        :return:
        """
        params = self._get_parameters()

        paramDescription = str(params)
        subscriberDescription = str(self._get_subscribers())
        publisherDescription = str(self._get_publishers())
        if self.verbosity:
            verbosityParam = '"' + self.verbosity + '"'
        else:
            verbosityParam = None

        #generate import statements
        imports = set()
        for subscriber in self._get_subscribers():
            imports.add("import {}".format(subscriber['import']))
        for publisher in self._get_publishers():
            imports.add("import {}".format(publisher['import']))
        imports = "\n".join(imports)

        # Read in template file
        templatefile = os.path.join(self.dynconfpath, "templates", "Interface.py.template")
        with open(templatefile, 'r') as f:
            template = f.read()

        content = Template(template).substitute(pkgname=self.pkgname, ClassName=self.classname, imports=imports,
                                                paramDescription=paramDescription,
                                                subscriberDescription=subscriberDescription,
                                                publisherDescription=publisherDescription,
                                                verbosityParam=verbosityParam,
                                                tfConfig=self.tf)

        py_file = os.path.join(self.py_gen_dir, "interface", self.classname + "Interface.py")
        try:
            if not os.path.exists(os.path.dirname(py_file)):
                os.makedirs(os.path.dirname(py_file))
        except OSError:
            # Stupid error, sometimes the directory exists anyway
            pass
        with open(py_file, 'w') as f:
            f.write(content)
        init_file = os.path.join(self.py_gen_dir, "interface", "__init__.py")
        with open(init_file, 'wa') as f:
            f.write("")

    def _generateyml(self):
        """
        Generate .yaml file for roslaunch
        :param self:
        :return:
        """
        params = self._get_parameters()

        content = "### This file was generated using the rosinterface_handler generate_yaml script.\n"

        for entry in params:
            if not entry["constant"]:
                content += "\n"
                content += "# Name:\t" + str(entry["name"]) + "\n"
                content += "# Desc:\t" + str(entry["description"]) + "\n"
                content += "# Type:\t" + str(entry["type"]) + "\n"
                if entry['min'] or entry['max']:
                    content += "# [min,max]:\t[" + str(entry["min"]) + "/" + str(entry["max"]) + "]" + "\n"
                if entry["global_scope"]:
                    content += "# Lives in global namespace!\n"
                if entry["default"] is not None:
                    content += str(entry["name"]) + ": " + str(entry["default"]) + "\n"
                else:
                    content += str(entry["name"]) + ": \n"

        yaml_file = os.path.join(os.getcwd(), self.classname + "Interface.yaml")

        with open(yaml_file, 'w') as f:
            f.write(content)

    def _get_parameters(self):
        """
        Returns parameter of this and all childs
        :return: list of all parameters
        """
        params = []
        params += self.parameters
        for child in self.childs:
            params.extend(child._get_parameters())
        return params

    def _get_subscribers(self):
        """
        Subscriber of this and all childs
        :return: list of all subscribers
        """
        subscribers = []
        subscribers.extend(self.subscribers)
        for child in self.childs:
            subscribers.extend(child._get_subscribers())
        return subscribers

    def _get_publishers(self):
        """
        Subscriber of this and all childs
        :return: list of all publishers
        """
        publishers = []
        publishers.extend(self.publishers)
        for child in self.childs:
            publishers.extend(child._get_publishers())
        return publishers

    def _get_root(self):
        """
        Returns the root object (for groups)
        :return:
        """
        if not self.parent:
            return self
        return self.parent._get_root()

    def _generate_param_entries(self):
        """
        Generates the entries for the cfg-file
        :return: list of param entries as string
        """
        param_entries = []
        dynamic_params = [p for p in self.parameters if p["configurable"]]

        if self.parent:
            param_entries.append(Template("$group_variable = $parent.add_group('$group')").substitute(
                group_variable=self.group_variable,
                group=self.group,
                parent=self.parent.group_variable))

        for enum in self.enums:
            param_entries.append(Template("$name = gen.enum([").substitute(
                name=enum['name'],
                parent=self.group))
            paramtype = self._pytype(enum['paramtype'])
            for value in enum['values']:
                enum_val = enum['values'].index(value) if paramtype == 'int' else "'" + value + "'"
                param_entries.append(
                    Template("    gen.const(name='$name', type='$type', value=$value, descr='$descr'),")
                        .substitute(name=value, type=paramtype, value=enum_val, descr=""))
            param_entries.append(Template("    ], '$description')").substitute(description=enum["description"]))

        for param in dynamic_params:
            content_line = Template("$group_variable.add(name = '$name', paramtype = '$paramtype', level = $level, "
                                    "description = '$description', edit_method=$edit_method").substitute(
                group_variable=self.group_variable,
                name=param["name"],
                paramtype=param['pytype'],
                level=param['level'],
                edit_method=param['edit_method'],
                description=param['description'])
            if param['default'] is not None:
                content_line += Template(", default=$default").substitute(default=self._get_pyvalue(param, "default"))
            if param['min'] is not None:
                content_line += Template(", min=$min").substitute(min=param['min'])
            if param['max'] is not None:
                content_line += Template(", max=$max").substitute(max=param['max'])
            content_line += ")"
            param_entries.append(content_line)

        for child in self.childs:
            param_entries.extend(child._generate_param_entries())
        return param_entries

    @staticmethod
    def _make_bool(param):
        if isinstance(param, bool):
            return param
        else:
            # Pray and hope that it is a string
            return bool(param)


# Create derived class for yaml generation
class YamlGenerator(InterfaceGenerator):
    def _generateImpl(self):
        self._generateyml()
        return 0
