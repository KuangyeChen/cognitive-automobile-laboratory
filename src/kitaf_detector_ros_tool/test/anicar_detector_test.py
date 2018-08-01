#!/usr/bin/env python
PKG = "kitaf_detector_ros_tool"

import sys

import unittest
import rospy
import rosnode
from util_testing_ros.listener import Listener
from util_testing_ros.initialization import *


class TestAnicarDetector(unittest.TestCase):
    def test_node_online(self):
        """
        Test that node is launched correctly
        """
        self.assertTrue(wait_until_online('/anicar_detector'), 'Could not contact node. Did it crash?')


    def test_subscribe_and_advertise(self):
        # subscribe to messages and wait for node to initialize
        # publisher = rospy.Publisher(rospy.get_param("/anicar_detector/subscriber_msg_name"), Header, queue_size=1)
        # listener = Listener(rospy.get_param("/anicar_detector/publisher_msg_name"), Header)
        # self.assertTrue(wait_for_initialization(publisher), "anicar_detector failed to subscribe to {}".format(publisher.name))

        # now do something like publishing messages and wait for response...
        # publisher.publish(mymsg)
        # self.assertIsNotNone(listener.wait_for_message(), "no answer from node")

        # ... or reconfigure the node
        # client = Client("anicar_detector", timeout=5)
        # client.update_configuration({"myparam": 5}) # blocks until update is confirmed
        pass



if __name__ == '__main__':
    import rostest

    rospy.init_node('anicar_detector_test')
    rostest.rosrun(PKG, 'anicar_detector_test', TestAnicarDetector)
