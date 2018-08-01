import rospy
import logging


logger_levels = {
    "debug": logging.DEBUG,
    "info": logging.INFO,
    "warn": logging.WARNING,
    "error": logging.ERROR,
    "fatal": logging.CRITICAL,
}


def set_logger_level():
    log_level = logger_levels[rospy.get_param("~verbosity", default="info").lower()]
    logging.getLogger('rosout').setLevel(log_level)
