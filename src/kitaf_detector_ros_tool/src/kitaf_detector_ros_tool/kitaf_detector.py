from __future__ import print_function, division
import rospy
from dynamic_reconfigure.server import Server

from kitaf_detector_ros_tool.cfg import KitafDetectorConfig
from kitaf_detector_ros_tool.interface.KitafDetectorInterface import KitafDetectorInterface

import cv2
import tensorflow as tf
import numpy as np
from detection_utils import *

import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Float64
from cv_bridge import CvBridge, CvBridgeError

__all__ = ["KitafDetector"]


class KitafDetector:
    def filter_sign(self, direction, direction_score, stop, stop_score):
        """
        Filter traffic sign signal

        :param direction:
        :param direction_score:
        :param stop:
        :param stop_score:
        :return:
        """

        rospy.logdebug("State: left:%.4f right:%.4f forward:%.4f stop:%.4f",
                       self.sign_state[DetectionClass.LEFT], self.sign_state[DetectionClass.RIGHT],
                       self.sign_state[DetectionClass.FORWARD], self.sign_state[DetectionClass.STOP])

        for key in self.sign_state:
            if stop and key == DetectionClass.STOP:
                update = stop_score
            elif direction and key == DetectionClass(direction):
                update = direction_score
            else:
                update = 1 - self.interface.model_confidence

            log_odd = np.log(update / (1 - update))
            self.sign_state[key] = self.sign_state[key] + log_odd
            # Truncate log odd.
            if self.sign_state[key] < -2:
                self.sign_state[key] = -2
            elif self.sign_state[key] > 20:
                self.sign_state[key] = 20

    def callback_synchronizer(self, color_image_msg, depth_image_msg):
        """
        Called when synchronized new messages arrives

        :param color_image_msg:
        :param depth_image_msg:
        :return:
        """
        if self.task == 1:
            rospy.loginfo_throttle(60, "In final task, No detection")
            return

        # Skip frames
        self.skip_count = (self.skip_count + 1) % self.interface.process_rate
        if self.skip_count != 0:
            return
        rospy.logdebug("----------------- New Frame -----------------")
        start_time = rospy.get_time()
        block_start_time = rospy.get_time()

        # Run detection
        color_image = self.bridge.imgmsg_to_cv2(color_image_msg, desired_encoding="rgb8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg) / 1000.0
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)
        block_start_time = log_time_consumption(block_start_time, "Get data")

        image_expand = np.expand_dims(color_image, axis=0)
        boxes, scores, classes, num = self.tf_session.run([self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                                                          feed_dict={self.image_tensor: image_expand})
        block_start_time = log_time_consumption(block_start_time, "TensorFlow")

        near_sign_id, near_sign_distance, near_sign_score = None, np.inf, None
        stop, stop_distance, stop_score = False, 10000.0, 0

        # Process detected objects
        for i in range(num):
            score = float(scores[0][i])
            if score < self.interface.detect_score_threshold:
                break

            class_id = int(classes[0][i])
            box = [int(boxes[0][i][0] * color_image.shape[0]), int(boxes[0][i][1] * color_image.shape[1]),
                   int(boxes[0][i][2] * color_image.shape[0]), int(boxes[0][i][3] * color_image.shape[1])]
            depth_crop, hsv_crop = crop_with_box([depth_image, hsv_image], box)

            mask = filter_color_depth(depth_crop, hsv_crop, class_id, self.interface.depth_threshold)

            if not mask.any():
                rospy.loginfo('Ignore %s detection: Too far or without depth info.', DetectionClass(class_id).name)
                continue

            distance = get_distance(depth_crop, mask, percent=self.interface.distance_percentile)

            if DetectionClass(class_id) == DetectionClass.STOP and distance < stop_distance:
                stop = True
                stop_score = score
                stop_distance = distance
                rospy.logdebug("!!stop : %f", stop_distance)
            else:
                if distance < near_sign_distance:
                    near_sign_id = class_id
                    near_sign_score = score
                    near_sign_distance = distance

            if self.interface.publish_image:
                draw_box(color_image, DetectionClass(class_id).name, distance, box)
        block_start_time = log_time_consumption(block_start_time, "Loop process")

        # Publish traffic_sign
        self.filter_sign(near_sign_id, near_sign_score, stop, stop_score)
        publish_sign(self.sign_state, self.interface.sign_score_threshold, stop_distance,
                     self.direction_publisher, self.stop_publisher)
        log_time_consumption(block_start_time, "Publish sign")
        log_time_consumption(start_time, "ALL PROCESS")

        # Publish image
        if self.interface.publish_image:
            # cv2.putText(color_image, 'FPS: ' + str(fps), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(color_image, "Cost: %.4fs" % (rospy.get_time() - start_time), (50, 50),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
            publish_image(color_image, self.bridge, self.image_publisher, encoding="rgb8")

    def reconfigure_callback(self, config, level):
        """
        Called when someone reconfigures this node (or at startup)
        """

        self.interface.from_config(config)
        return config

    def task_callback(self, msg):
        self.task = msg.data

    def __init__(self):
        """
        Sets up ros stuff
        """

        # Initialization
        self.interface = KitafDetectorInterface()
        self.bridge = CvBridge()
        self.skip_count = 0
        self.sign_state = {DetectionClass.LEFT: 0.0,
                           DetectionClass.RIGHT: 0.0,
                           DetectionClass.FORWARD: 0.0,
                           DetectionClass.STOP: 0.0}
        self.task = 0

        # TensorFlow initialization
        try:
            detection_graph = tf.Graph()
            with detection_graph.as_default(), tf.gfile.GFile(rospy.get_param("~model"), 'rb') as fid:
                od_graph_def = tf.GraphDef()
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

                self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                self.d_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                self.d_scores = detection_graph.get_tensor_by_name('detection_scores:0')
                self.d_classes = detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_d = detection_graph.get_tensor_by_name('num_detections:0')
                self.tf_session = tf.Session(graph=detection_graph)
        except tf.errors.OpError:
            rospy.logerr('TensorFlow saved model not found or cannot be loaded.')
            rospy.signal_shutdown('TensorFlow saved model not found or cannot be loaded.')
        else:
            rospy.loginfo('TensorFlow initialized.')

        # Register callbacks for subscriber and dynamic reconfigure
        self.reconfigure = Server(KitafDetectorConfig, self.reconfigure_callback)
        self.color_image_subscriber = message_filters.Subscriber(self.interface.color_image_topic, Image)
        self.depth_image_subscriber = message_filters.Subscriber(self.interface.depth_image_topic, Image)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.color_image_subscriber,
                                                                         self.depth_image_subscriber],
                                                                        queue_size=5, slop=0.1)
        self.synchronizer.registerCallback(self.callback_synchronizer)
        self.task_subscriber = rospy.Subscriber("/current_task", Int8, self.task_callback, queue_size=5)

        # Publishers
        self.image_publisher = rospy.Publisher(self.interface.image_publish_topic, Image, queue_size=5)
        self.direction_publisher = rospy.Publisher(self.interface.direction_publish_topic, Int8, queue_size=5)
        self.stop_publisher = rospy.Publisher(self.interface.stop_publish_topic, Float64, queue_size=5)
