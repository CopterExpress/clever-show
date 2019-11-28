import rospy
import sys
import time
import logging
import ConfigParser
from clever.srv import SetAttitude
from sensor_msgs.msg import Range
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

config = ConfigParser.ConfigParser()
config.read("client_config.ini")

visual_pose_timeout = config.getfloat('VISUAL_POSE_WATCHDOG', 'timeout')
timeout_action = config.get('VISUAL_POSE_WATCHDOG', 'action')
emergency_land_thrust = config.getfloat('VISUAL_POSE_WATCHDOG', 'emergency_land_thrust')
emergency_land_decrease_thrust_after = config.getfloat('VISUAL_POSE_WATCHDOG', 'emergency_land_decrease_thrust_after')
timeout_to_disarm_after_watchdog_action = config.getfloat('VISUAL_POSE_WATCHDOG', 'timeout_to_disarm_after_watchdog_action')

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
logger.setLevel(logging.INFO)
logger.addHandler(handler)

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_attitude = rospy.ServiceProxy('/set_attitude', SetAttitude)

visual_pose_last_timestamp = 0
armed = False
mode = ''
laser_range = 10
emergency = False

rospy.init_node('visual_pose_watchdog')
logger.info('visual_pose_watchdog inited')
logger.info('timeout = {} | timeout_action = {}'.format(visual_pose_timeout, timeout_action))
logger.info('timeout_to_disarm_after_watchdog_action = {}'.format(timeout_to_disarm_after_watchdog_action))
if timeout_action == 'emergency_land':
    logger.info('emergency_land_thrust: {}'.format(emergency_land_thrust))

rate = rospy.Rate(10)

def visual_pose_callback(data):
    global visual_pose_last_timestamp
    visual_pose_last_timestamp = data.header.stamp.to_sec()

def state_callback(data):
    global armed, mode
    armed = data.armed
    mode = data.mode

def laser_callback(data):
    global laser_range
    laser_range = data.range

def watchdog_callback(event):
    global visual_pose_last_timestamp, armed, mode, timeout_action, laser_range, emergency
    logger.debug("armed: {} | mode: {} | delta: {} | action: {} | range: {}".format(armed, mode, abs(time.time() - visual_pose_last_timestamp), timeout_action, laser_range))
    if abs(time.time() - visual_pose_last_timestamp) > visual_pose_timeout:
        if armed:
            if timeout_action in ['land', 'emergency_land', 'disarm']:
                emergency = True
            if timeout_action == 'land':
                logger.info('Visual pose data is too old, copter is armed, landing...')
                while mode != "AUTO.LAND":
                    try:
                        set_mode(custom_mode='AUTO.LAND')
                    except rospy.ServiceException as e:
                        logger.info(e)
                    rate.sleep()
                logger.info('Land mode is set')
                action_timestamp = time.time()
                while armed:
                    if time.time() - action_timestamp > timeout_to_disarm_after_watchdog_action:
                        try:
                            arming(False)
                        except rospy.ServiceException as e:
                            logger.info(e)   
                    rate.sleep()
            elif timeout_action == 'disarm':
                logger.info('Visual pose data is too old, copter is armed, disarming...')
                while armed:
                    try:
                        arming(False)
                    except rospy.ServiceException as e:
                        logger.info(e)   
                    rate.sleep() 
            elif timeout_action == 'emergency_land':
                logger.info('Visual pose data is too old, copter is armed, emergency landing...')
                action_timestamp = time.time()
                current_thrust = emergency_land_thrust
                while armed:
                    logger.debug("Emergency land | range: {} | thrust: {}".format(laser_range, current_thrust))
                    try:
                        set_attitude(thrust = current_thrust, yaw = 0, frame_id = 'body')
                    except rospy.ServiceException as e:
                        logger.info(e) 
                    delta = time.time() - action_timestamp
                    if delta > timeout_to_disarm_after_watchdog_action:
                        try:
                            arming(False)
                        except rospy.ServiceException as e:
                            logger.info(e)  
                    if (laser_range < 0.1 or delta > emergency_land_decrease_thrust_after) and current_thrust > 0.:
                        current_thrust -= 0.02
                        if current_thrust < 0:
                            current_thrust = 0  
                    rate.sleep()
            logger.info('Disarmed') 
            emergency = False                
        else:
            logger.info('Visual pose data is too old')

rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, visual_pose_callback)

rospy.Subscriber('/mavros/state', State, state_callback)

rospy.Subscriber('/mavros/distance_sensor/rangefinder', Range, laser_callback)

emergency_pub = rospy.Publisher('/emergency', Bool, queue_size=10)

rospy.Timer(rospy.Duration(0.5), watchdog_callback)

while not rospy.is_shutdown():
    emergency_msg = Bool()
    emergency_msg.data = emergency
    emergency_pub.publish(emergency_msg)
    rate.sleep()
    
