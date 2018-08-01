#!/usr/bin/env python
PKG = 'util_testing_ros'

import rospy
import time
import sys
import unittest
from std_msgs.msg import Header
from util_testing_ros.time_controller import TimeController
from util_testing_ros.listener import Listener
from util_testing_ros.initialization import *

# fake node responds to every message (on /out) with the current time (on /in)
class FakeNode:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/out", Header, self.on_message, queue_size=5)
        self.publisher = rospy.Publisher("/in", Header, queue_size=1)

    def on_message(self, msg):
        time.sleep(0.05)
        header = Header()
        header.stamp = rospy.Time.now()
        self.publisher.publish(header)


class TestUtilTestingRosPython(unittest.TestCase):
    def test_time_controller(self):
        rospy.set_param("/use_sim_time", False)

        with self.assertRaises(AssertionError):
            TimeController()

        rospy.set_param("/use_sim_time", True)

        tc = TimeController(rospy.Time(5))
        time.sleep(0.05)
        self.assertAlmostEqual(rospy.Time.now(), rospy.Time(5))

        tc.advance(rospy.Duration(1))
        time.sleep(0.05)
        self.assertAlmostEqual(tc.time_now, rospy.Time.now())

    def test_initialization(self):
        publisher = rospy.Publisher("/out", Header, queue_size=1)
        self.assertFalse(wait_for_initialization(publisher))

        fakeNode = FakeNode()
        self.assertTrue(wait_for_initialization(publisher))

    def test_listener(self):
        listener = Listener("/in", Header)

        self.assertFalse(listener.wait_for_message())

        fakeNode = FakeNode()

        rospy.set_param("/use_sim_time", True)
        publisher = rospy.Publisher("/out", Header, queue_size=1)
        tc = TimeController(rospy.Time(5))

        self.assertTrue(wait_for_initialization(publisher))
        tc.advance(rospy.Duration(1))
        publisher.publish(Header())
        self.assertIsNotNone(listener.wait_for_message())
        self.assertAlmostEqual(listener.msg.stamp, rospy.Time.now())
        print("Jo")


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_bias_python')
    rostest.rosrun(PKG, 'test_util_testing_ros_python', TestUtilTestingRosPython)