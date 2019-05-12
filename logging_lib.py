import logging

try:
    import rospy
except ImportError:
    ros = False
else:
    ros = True


class Logger:
    def __init__(self, logger=logging.getLogger(), use_ros=False):
        self.ros = True if use_ros and ros else False
        self.logger = logger

    def info(self, msg):
        self.logger.info(msg)

        if self.ros:
            rospy.loginfo(msg)

    def debug(self, msg):
        self.logger.debug(msg)

        if self.ros:
            rospy.logdebug(msg)

    def warning(self, msg):
        self.logger.warning(msg)

        if self.ros:
            rospy.logwarn(msg)

    def error(self, msg):
        self.logger.error(msg)

        if self.ros:
            rospy.logerr(msg)

    def critical(self, msg):
        self.logger.critical(msg)

        if self.ros:
            rospy.logfatal(msg)
