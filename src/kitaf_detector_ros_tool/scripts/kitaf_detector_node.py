#!/usr/bin/env python
from kitaf_detector_ros_tool.kitaf_detector import KitafDetector
import sys
import rospy

def main(args):
    """
    Initializes and cleanup ros node
    """
    rospy.init_node('kitaf_detector_node')
    kitaf_detector = KitafDetector()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
