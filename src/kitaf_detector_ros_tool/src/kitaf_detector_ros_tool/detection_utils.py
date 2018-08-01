import rospy
import tf2_ros
import numpy as np
import quaternion
import cv2
from sensor_msgs.msg import PointCloud2, PointField
from enum import Enum

from std_msgs.msg import Int8, Float64

__all__ = ["publish_image",
           "get_distance",
           "filter_color_depth",
           "draw_box",
           "publish_sign",
           "DetectionClass",
           "crop_with_box",
           "log_time_consumption",
           "get_camera_pose",
           "get_tf_pose",
           "get_points",
           "publish_points"]


class DetectionClass(Enum):
    """
    Definitions for classes of detected objects.
    """

    SEE_NO_TRAFFIC_SIGN = 0
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    STOP = 4


def publish_points(points, publisher, frame_id=None, stamp=None):
    """
    Publish point cloud.

    :param points:
    :param publisher:
    :param frame_id:
    :param stamp:
    :return:
    """
    if not points.dtype == np.float32:
        rospy.logwarn("Point cloud type not float32, may not be visualized by Rviz.")

    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)

    msg.is_dense = int(np.isfinite(points).all())
    msg.data = points.tostring()

    publisher.publish(msg)


def get_points(depth, box, translation, rotation, camera_info):
    """
    Calculate point cloud in wanted frame(provided as translation and rotation).

    :param depth:
    :param translation:
    :param rotation:
    :param camera_info:
    :return:
    """

    (fx, __, cx, tx,
     __, fy, cy, ty,
     __, __, __, __) = camera_info.P

    u = np.arange(box[1], box[3]).reshape(1, -1)
    x = (u * depth - cx * depth - tx) / fx

    v = np.arange(box[0], box[2]).reshape(-1, 1)
    y = (v * depth - cy * depth - ty) / fy

    points_camera = np.stack([x, y, depth], axis=-1)
    points_world = np.add(np.matmul(rotation, points_camera.reshape(-1, 3).T), translation, dtype=np.float32)

    return points_world.T


def get_tf_pose(tf_buffer, target_frame, source_frame, target_time=rospy.Time()):
    try:
        trans = tf_buffer.lookup_transform(target_frame, source_frame, target_time)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Can not get camera position: %s" % e.message)
        return None
    return trans


def get_camera_pose(tf_buffer, map_frame_id, camera_frame_id, target_time=rospy.Time()):
    trans = get_tf_pose(tf_buffer, map_frame_id, camera_frame_id, target_time)

    if trans is None:
        return None, None

    translation = np.asarray([trans.transform.translation.x,
                              trans.transform.translation.y,
                              trans.transform.translation.z]).reshape(-1, 1)
    rotation = quaternion.as_rotation_matrix(quaternion.quaternion(trans.transform.rotation.w,
                                                                   trans.transform.rotation.x,
                                                                   trans.transform.rotation.y,
                                                                   trans.transform.rotation.z))
    return translation, rotation


def log_time_consumption(time_stamp, block_name):
    """
    Log time consumption to debug level.

    :param time_stamp:
    :param block_name:
    :return:
    """

    rospy.logdebug("%s cost %f seconds", block_name, rospy.get_time() - time_stamp)
    return rospy.get_time()


def publish_sign(state, threshold, stop_distance, direction_publisher, stop_publisher):
    """
    Publish the most likely traffic sign signal.
    
    :param state: 
    :param threshold:
    :param stop_distance:
    :param direction_publisher: 
    :param stop_publisher: 
    :return: 
    """

    direction_states = {sign: state[sign] for sign in [DetectionClass.LEFT,
                                                       DetectionClass.RIGHT,
                                                       DetectionClass.FORWARD]}
    most_likely_direction = max(direction_states, key=direction_states.get)
    if state[most_likely_direction] < threshold:
        new_msg = Int8(DetectionClass.SEE_NO_TRAFFIC_SIGN.value)
    else:
        new_msg = Int8(most_likely_direction.value)
    direction_publisher.publish(new_msg)

    if state[DetectionClass.STOP] < threshold:
        new_msg = Float64(10000.0)
    else:
        new_msg = Float64(stop_distance)
    stop_publisher.publish(new_msg)


def publish_image(image, bridge, publisher, encoding="passthrough"):
    """
    Publish an annotated image.

    :param image:
    :param bridge:
    :param publisher:
    :param encoding:
    :return:
    """

    new_msg = bridge.cv2_to_imgmsg(image, encoding)
    publisher.publish(new_msg)


def draw_box(image, class_name, distance, box):
    """
    Draw a box around a detection, with class name and distance.
    And Draw the nearest pixel if possible.

    :param image:
    :param class_name:
    :param distance:
    :param box:
    :return:
    """

    (ymin, xmin, ymax, xmax) = box

    cv2.putText(image, class_name, (xmin, ymin),
                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(image,  '%.2f m' % distance, (xmin, ymin - 30),
                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 2)
    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (125, 255, 51), thickness=2)


def filter_color_depth(depth_image, hsv_image, class_id, depth_threshold):
    """
    Get binary mask of image, filtering out points with wrong color or too far away.

    :param depth_image:
    :param hsv_image:
    :param class_id:
    :param depth_threshold:
    :return:
    """

    mask = np.ones(depth_image.shape, dtype=bool)

    mask[depth_image > depth_threshold] = False
    mask[depth_image == 0.0] = False

    if DetectionClass(class_id) == DetectionClass.STOP:
        segment = cv2.bitwise_or(cv2.inRange(hsv_image, (0, 43, 20), (10, 255, 255)),
                                 cv2.inRange(hsv_image, (165, 43, 20), (180, 255, 255)))
    else:
        segment = cv2.inRange(hsv_image, (95, 43, 20), (130, 255, 255))
    mask[segment == 0] = False

    return mask


def crop_with_box(arrays, box):
    """
    Crop a list of array with one box, without copy.

    :param arrays:
    :param box:
    :return:
    """

    return [array[box[0]: box[2], box[1]: box[3]] for array in arrays]


def get_distance(depth_image, mask, percent=50):
    """
    Get distance in depth image filtered by mask.
    The distance returned is percentile in depth image.

    :param depth_image:
    :param mask:
    :param percent:
    :return:
    """
    return np.percentile(depth_image[mask], percent)
