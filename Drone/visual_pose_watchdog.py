import rospy
import sys
import time
import logging
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

logging.basicConfig(  # TODO all prints as logs
   level=logging.DEBUG, # INFO
   stream=sys.stdout,
   format="%(asctime)s [%(name)-7.7s] [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s",
   handlers=[
       logging.StreamHandler(sys.stdout),
   ])

handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s [%(name)-7.7s] [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
handler.setFormatter(formatter)

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(handler)

land = rospy.ServiceProxy('/land', Trigger)

visual_pose_last_timestamp = 0
visual_pose_timeout = 2.

rospy.init_node('visual_pose_watchdog')
logger.info('visual_pose_watchdog inited')

def visual_pose_callback(data):
    global visual_pose_last_timestamp
    visual_pose_last_timestamp = data.header.stamp.to_sec()

def watchdog_callback(event):
    global visual_pose_last_timestamp
    if (time.time() - visual_pose_last_timestamp) > visual_pose_timeout and visual_pose_last_timestamp != 0:
        logger.info('Visual pose data is too old, landing...')
        land()

rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, visual_pose_callback)

rospy.Timer(rospy.Duration(0.5), watchdog_callback)

rospy.spin()
    
