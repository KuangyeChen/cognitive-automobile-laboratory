#!/usr/bin/env python

PKG = 'rosinterface_handler'
import unittest
import rospy
import tests

if __name__ == '__main__':
    import rostest

    rospy.init_node(PKG)
    rostest.rosrun(PKG, "RosinterfaceTestSuite", "tests.RosinterfaceTestSuite")
