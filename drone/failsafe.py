import rospy
import os
import sys
import time
import math
import logging
import threading

# for backward compatibility with clever
try:
    from clever import srv
except ImportError:
    from clover import srv

from sensor_msgs.msg import Range
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = (os.path.dirname(os.path.realpath(__file__)))
lib_dir = os.path.realpath(os.path.join(current_dir, '../lib'))
sys.path.insert(0, lib_dir)

from config import ConfigManager

config = ConfigManager()
config.load_config_and_spec("config/client.ini")

watchdog_is_enabled = config.failsafe_enabled
log_state = config.failsafe_log_state
vision_pose_delay_after_arm = config.failsafe_vision_pose_delay_after_arm
visual_pose_timeout = config.failsafe_vision_pose_timeout
pos_delta_max = config.failsafe_position_delta_max
watchdog_action = config.failsafe_action
timeout_to_disarm = config.failsafe_disarm_timeout
emergency_land_thrust = config.emergency_land_thrust
emergency_land_decrease_thrust_after = config.emergency_land_decrease_thrust_after

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

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_attitude = rospy.ServiceProxy('/set_attitude', srv.SetAttitude)

visual_pose_last_timestamp = 0
armed = False
mode = ''
laser_range = 10
emergency = False

local_pose = None
setpoint_raw = None
setpoint_position = None
setpoint_pose = None

arm_start_time = None
offboard_start_time = None
offboard_disarmed_timeout = 3.

emergency_land_called = False

rospy.init_node('visual_pose_watchdog')
logger.info('visual_pose_watchdog inited')
logger.info('visual_pose_timeout = {} | position_delta_max = {} | watchdog_action = {}'.format(visual_pose_timeout, pos_delta_max, watchdog_action))
logger.info('timeout_to_disarm = {}'.format(timeout_to_disarm))
if watchdog_action == 'emergency_land':
    logger.info('emergency_land_thrust: {}'.format(emergency_land_thrust))

rate = rospy.Rate(10)

def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def get_pos_delta(PoseStamped1, PoseStamped2):
    if PoseStamped1 is None or PoseStamped2 is None:
        return float('nan')
    pos1 = PoseStamped1.pose.position
    pos2 = PoseStamped2.pose.position
    return get_distance(pos1.x, pos1.y, pos1.z, pos2.x, pos2.y, pos2.z)

def get_time_delta(PoseStamped1, PoseStamped2):
    if PoseStamped1 is None or PoseStamped2 is None:
        return float('nan')
    time1 = PoseStamped1.header.stamp.to_sec()
    time2 = PoseStamped2.header.stamp.to_sec()
    return time1 - time2

def visual_pose_callback(data):
    global visual_pose_last_timestamp
    visual_pose_last_timestamp = data.header.stamp.to_sec()

def local_pose_callback(data):
    global local_pose
    local_pose = data

def setpoint_raw_callback(data):
    global setpoint_raw, setpoint_position, setpoint_pose
    setpoint_raw_pose = PoseStamped()
    setpoint_raw_pose.header = data.header
    setpoint_raw_pose.pose.position = data.position
    setpoint_raw = setpoint_raw_pose
    setpoint_pose = get_current_setpoint_pose(setpoint_raw, setpoint_position)

def setpoint_position_callback(data):
    global setpoint_raw, setpoint_position, setpoint_pose
    setpoint_position = data
    setpoint_pose = get_current_setpoint_pose(setpoint_raw, setpoint_position)

def get_current_setpoint_pose(_setpoint_raw, _setpoint_position):
    if _setpoint_position is None and _setpoint_raw is None:
        return None
    elif _setpoint_position is not None and _setpoint_raw is None:
        return _setpoint_position
    elif _setpoint_raw is not None and _setpoint_position is None:
        return _setpoint_raw
    else:
        return _setpoint_raw if _setpoint_raw.header.stamp > _setpoint_position.header.stamp else _setpoint_position

def state_callback(data):
    global armed, mode
    armed = data.armed
    mode = data.mode

def laser_callback(data):
    global laser_range
    laser_range = data.range

def emergency_land(disarm_if_timeout = True):
    global emergency_land_thrust, laser_range
    current_thrust = emergency_land_thrust
    action_timestamp = time.time()
    while armed:
        logger.debug("Emergency land | range: {:.2f} | thrust: {:.2f}".format(laser_range, current_thrust))
        if current_thrust >= 0.03:
            try:
                set_attitude(thrust = current_thrust, yaw = 0, frame_id = 'body', auto_arm = True)
            except rospy.ServiceException as e:
                logger.info(e)
        delta = time.time() - action_timestamp
        if delta > timeout_to_disarm and disarm_if_timeout:
            try:
                arming(False)
            except rospy.ServiceException as e:
                logger.info(e)
        if (laser_range < 0.1 or delta > emergency_land_decrease_thrust_after) and current_thrust >= 0.:
            current_thrust -= 0.02
            if current_thrust <= 0.03:
                current_thrust = 0
                try:
                    arming(False)
                except rospy.ServiceException as e:
                    logger.info(e)
        rate.sleep()

def emergency_land_service(request):
    global emergency_land_called, armed
    responce = TriggerResponse()
    if armed:
        responce.success = True
        responce.message = "Start emergency landing"
        emergency_land_called = True
    else:
        responce.success = False
        responce.message = "Copter is disarmed, no need for emergency landing!"
        emergency_land_called = False
    return responce

def watchdog_callback(event):
    global visual_pose_last_timestamp, armed, mode, watchdog_action, laser_range
    global emergency, local_pose, setpoint_pose, emergency_land_called, log_state
    global offboard_start_time, arm_start_time, vision_pose_delay_after_arm
    pos_delta = get_pos_delta(local_pose, setpoint_pose)
    pos_dt = get_time_delta(local_pose, setpoint_pose)
    visual_pose_dt = abs(time.time() - visual_pose_last_timestamp)
    if log_state:
        logger.info("armed: {} | mode: {} | vis_dt: {:.2f} | pos_delta: {:.2f} | pos_dt: {:.2f} | range: {:.2f} | watchdog_action: {}".format(
                    armed, mode, visual_pose_dt, pos_delta, pos_dt, laser_range, watchdog_action))
    if mode == 'OFFBOARD':
        if offboard_start_time is None:
            offboard_start_time = time.time()
        if armed:
            if arm_start_time is None:
                arm_start_time = time.time()
            arm_time = time.time() - arm_start_time
            logger.debug('arm time: {}'.format(arm_time))
            if arm_time > vision_pose_delay_after_arm and watchdog_is_enabled:
                if (visual_pose_dt > visual_pose_timeout and visual_pose_timeout != 0.) or (pos_delta > pos_delta_max and pos_delta_max != 0.):
                    action_timestamp = time.time()
                    emergency = True
                    if watchdog_action not in ['land', 'emergency_land', 'disarm']:
                        watchdog_action = 'land'
                    if watchdog_action == 'land':
                        logger.info('Visual pose data is too old, copter is armed, landing...')
                        while mode != "AUTO.LAND":
                            try:
                                set_mode(custom_mode='AUTO.LAND')
                            except rospy.ServiceException as e:
                                logger.info(e)
                            if time.time() - action_timestamp > timeout_to_disarm:
                                break
                            rate.sleep()
                        else:
                            logger.info('Land mode is set')
                        while armed:
                            if time.time() - action_timestamp > timeout_to_disarm:
                                try:
                                    arming(False)
                                except rospy.ServiceException as e:
                                    logger.info(e)
                            rate.sleep()
                    elif watchdog_action == 'disarm':
                        logger.info('Visual pose data is too old, copter is armed, disarming...')
                        while armed:
                            try:
                                arming(False)
                            except rospy.ServiceException as e:
                                logger.info(e)
                            rate.sleep()
                    elif watchdog_action == 'emergency_land':
                        if visual_pose_dt > visual_pose_timeout:
                            logger.info('Visual pose data is too old, copter is armed, emergency landing...')
                        if pos_delta > pos_delta_max:
                            logger.info('Position delta is {} m, copter is armed, emergency landing...'.format(pos_delta))
                        emergency_land()
                    logger.info('Disarmed')
                    emergency = False
            if emergency_land_called:
                emergency = True
                logger.info('/emergency_land service was called, start emergency landing...')
                emergency_land()
                logger.info('Disarmed')
                emergency = False
                emergency_land_called = False
        else:
            arm_start_time = None
            if time.time() - offboard_start_time > offboard_disarmed_timeout:
                try:
                    set_mode(custom_mode='AUTO.LAND')
                except rospy.ServiceException as e:
                    logger.info(e)
    else:
        offboard_start_time = None
        if (abs(time.time() - visual_pose_last_timestamp) > visual_pose_timeout and visual_pose_timeout != 0.0):
            logger.info('Visual pose data is too old')

rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, visual_pose_callback)

rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_pose_callback)

rospy.Subscriber('/mavros/setpoint_position/local', PoseStamped, setpoint_position_callback)

rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, setpoint_raw_callback)

rospy.Subscriber('/mavros/state', State, state_callback)

rospy.Subscriber('/mavros/distance_sensor/rangefinder', Range, laser_callback)

emergency_pub = rospy.Publisher('/emergency', Bool, queue_size=10)

rospy.Service('emergency_land', Trigger, emergency_land_service)

rospy.Timer(rospy.Duration(0.5), watchdog_callback)

while not rospy.is_shutdown():
    emergency_msg = Bool()
    emergency_msg.data = emergency
    emergency_pub.publish(emergency_msg)
    rate.sleep()
