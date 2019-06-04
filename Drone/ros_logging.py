import logging
import rospy


class RosHandler(logging.Handler):

    level_map = {
        logging.DEBUG: rospy.logdebug,
        logging.INFO: rospy.loginfo,
        logging.WARNING: rospy.logwarn,
        logging.ERROR: rospy.logerr,
        logging.CRITICAL: rospy.logfatal
    }

    def emit(self, record):
        print(record.levelno, record.name, record.msg)
        if "rosout" not in record.msg:
            try:
                pass
                #self.level_map[record.levelno]("%s: %s" % (record.name, record.msg))
            except KeyError:
                rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno, record.name, record.msg))


def route_logger_to_ros(logger_name=None):
    if logger_name is not None:
        logging.getLogger(logger_name).addHandler(RosHandler())
    else:
        logging.getLogger().addHandler(RosHandler())
