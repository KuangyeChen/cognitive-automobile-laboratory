import rospy
import tf2_ros
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from cv_bridge import CvBridge, CvBridgeError

import cv2
import tensorflow as tf
import numpy as np

from detection_utils import *

from kitaf_detector_ros_tool.cfg import AnicarDetectorConfig
from kitaf_detector_ros_tool.interface.AnicarDetectorInterface import AnicarDetectorInterface

class AnicarDetector:
    def callback_synchronizer(self, color_image_msg, depth_image_msg, camera_info_msg):
        """
        Called when a new message arrives
        """
        translation, rotation = get_camera_pose(self.tfBuffer,
                                                self.interface.map_frame_id,
                                                camera_info_msg.header.frame_id)
        vehicle_pose = get_tf_pose(self.tfBuffer,
                                   self.interface.map_frame_id,
                                   self.interface.vehicle_frame_id)

        if translation is None or vehicle_pose is None:
            return

        vehicle_position = np.array([vehicle_pose.transform.translation.x, vehicle_pose.transform.translation.y])

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
        block_start_time = log_time_consumption(block_start_time, "Get data")

        image_expand = np.expand_dims(color_image, axis=0)
        boxes, scores, classes, num = self.tf_session.run([self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                                                          feed_dict={self.image_tensor: image_expand})
        block_start_time = log_time_consumption(block_start_time, "TensorFlow")

        if float(scores[0][0]) < self.interface.detect_score_threshold:
            rospy.loginfo("No anicar detected.")
            return

        box = [int(boxes[0][0][0] * color_image.shape[0]), int(boxes[0][0][1] * color_image.shape[1]),
               int(boxes[0][0][2] * color_image.shape[0]), int(boxes[0][0][3] * color_image.shape[1])]

        [depth_crop] = crop_with_box([depth_image], box)
        all_points = get_points(depth_crop, box, translation, rotation, camera_info_msg)

        all_points = all_points[np.logical_and(all_points[:, 2] > 0.01, all_points[:, 2] < 0.2)]

        distances = []
        points = []
        for point in all_points:
            tmp_dist = np.linalg.norm([point[0] - vehicle_position[0],
                                       point[1] - vehicle_position[1]])
            if tmp_dist < 0.17:
                continue
            points.append([point[0], point[1], point[2]])
            distances.append(tmp_dist)

        if len(distances) == 0:
            rospy.loginfo("No anicar detected.")
            return

        points = np.asarray(points, dtype=np.float32)
        sort_id = np.argpartition(distances, int(0.2 * len(points)))
        distance = distances[sort_id[int(0.2 * len(points))]]
        object_position = points[sort_id[int(0.2 * len(points))]][:2]

        new_pose = PoseStamped()
        new_pose.header.frame_id = "world"
        new_pose.header.stamp = rospy.Time.now()
        new_pose.pose.position.x, new_pose.pose.position.y = object_position
        self.pose_publisher.publish(new_pose)
        block_start_time = log_time_consumption(block_start_time, "get target position")

        draw_box(color_image, 'anicar', distance, box)
        block_start_time = log_time_consumption(block_start_time, "draw box")

        log_time_consumption(start_time, "ALL PROCESS")

        # Publish image
        if self.interface.publish_image:
            # Publish points
            publish_points(points, self.points_publisher, self.interface.map_frame_id, rospy.Time.now())
            block_start_time = log_time_consumption(block_start_time, "Publish PointCloud2")
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

    def __init__(self):
        """
        Sets up ros stuff
        """
        # Initialization
        self.interface = AnicarDetectorInterface()

        # tf
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.bridge = CvBridge()
        self.skip_count = 0

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
        self.color_image_subscriber = message_filters.Subscriber(self.interface.color_image_topic, Image)
        self.depth_image_subscriber = message_filters.Subscriber(self.interface.depth_image_topic, Image)
        self.camera_info_subscriber = message_filters.Subscriber(self.interface.camera_info_topic, CameraInfo)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer([self.color_image_subscriber,
                                                                         self.depth_image_subscriber,
                                                                         self.camera_info_subscriber],
                                                                        queue_size=5, slop=0.1)

        self.synchronizer.registerCallback(self.callback_synchronizer)

        # Publishers
        self.image_publisher = rospy.Publisher(self.interface.image_publish_topic, Image, queue_size=5)
        self.pose_publisher = rospy.Publisher(self.interface.pose_publish_topic, PoseStamped, queue_size=5)
        self.points_publisher = rospy.Publisher(self.interface.points_publish_topic, PointCloud2, queue_size=5)

        # Register callbacks for subscriber and dynamic reconfigure
        self.reconfigure = Server(AnicarDetectorConfig, self.reconfigure_callback)

