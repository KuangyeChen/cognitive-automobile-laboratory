import rospy
from rosgraph_msgs.msg import Clock


class TimeController:
    def __init__(self, t_init = rospy.Time(1)):
        """
        This class provides a simple way to fake the clock in ros for debugging or unittests.
        Be aware that nothin will happen if you do not publish clock updates regularily (nodelet managers will not
        load nodelets, subscriber not subscribe and publisher not publish
        :param t_init: Start time in ros (do not use 0!)
        """
        use_sim_time = rospy.get_param("/use_sim_time", False)

        if not use_sim_time:
            raise AssertionError("use_sim_time is not set. Can not control time!")
        self.time_publisher = rospy.Publisher("/clock", Clock, queue_size=1)
        self.time_now = t_init
        self.publish()


    def advance(self, delta = rospy.Duration(1)):
        """
        advance time by a delta and publish it
        :param delta: Duration: delta time
        """
        self.time_now += delta
        self.publish()


    def publish(self):
        """
        publish forces TimeController to publish the current time again
        """
        self.time_publisher.publish(self.time_now)
