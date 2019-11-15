import rospy
import time
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

land = rospy.ServiceProxy('/land', Trigger)

visual_pose_last_timestamp = 0
visual_pose_timeout = 1.

rospy.init_node('visual_pose_watchdog')
rospy.loginfo('visual_pose_watchdog inited')

def visual_pose_callback(data):
    global visual_pose_last_timestamp
    visual_pose_last_timestamp = data.header.stamp.to_sec()

def watchdog_callback(event):
    global visual_pose_last_timestamp
    if (time.time() - visual_pose_last_timestamp) > visual_pose_timeout and visual_pose_last_timestamp != 0:
        rospy.loginfo('Visual pose data is too old, landing...')
        land()

rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, visual_pose_callback)

rospy.Timer(rospy.Duration(0.5), watchdog_callback)

rospy.spin()
    