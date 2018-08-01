#!/usr/bin/env python
from kitaf_detector_ros_tool.anicar_detector import AnicarDetector
import sys
import rospy

def main(args):
    """
    Initializes and cleanup ros node
    """
    rospy.init_node('anicar_detector_node')
    anicar_detector = AnicarDetector()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
