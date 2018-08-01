import rospy
import rosnode
import math
import time


def wait_until_online(node_name, timeout=rospy.Duration(5)):
    """
    blocks until a node has become online and can be pinged
    :param node_name: name of the node (including namespace)
    :param timeout: time to wait until failure is reported
    :return: true if contact was made with the node
    """
    # wait until node is listed in rosmaster
    waitcount = 0
    online = False
    while waitcount < 5 and not rospy.is_shutdown():
        if node_name in rosnode.get_node_names():
            online = True
            break
        waitcount += 1
        rospy.sleep(timeout / 5.)
    if not online:
        return False

    # ping the node
    return rosnode.rosnode_ping(node_name, int(math.ceil(timeout.to_sec())))

def wait_for_initialization(publishers, timeout=rospy.Duration(2)):
    """
    blocks until a node has subscribed to all publishers
    :param subscribers: list with all publishers {publisher1, publisher2, ..}
    :param timeout: time to wait for nodes to subscribe
    :return: true if nodes managed to subscribe, false after timeout
    """
    if not isinstance(publishers, list):
        publishers = [publishers]

    waitcount = 0
    while waitcount < 20 and not rospy.is_shutdown():
        online = True
        waitcount += 1
        for publisher in publishers:
            online &= publisher.get_num_connections() >= 1
        time.sleep(timeout.to_sec()/20.)
        if online:
            return True
    return False