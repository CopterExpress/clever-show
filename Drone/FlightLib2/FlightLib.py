#!/usr/bin/python
from __future__ import print_function
import sys
import math
import time
import logging
import threading
import rospy
from clever import srv
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger

module_logger = logging.getLogger("FlightLib.FlightLib")

# create proxy service
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_rates = rospy.ServiceProxy('/set_rates', srv.SetRates)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
landing = rospy.ServiceProxy('/land', Trigger)

module_logger.info("Proxy services inited")

# globals
FREQUENCY = 1000/25  # HZ
TOLERANCE = 0.2

interrupt_event = threading.Event()


def interrupt():
    module_logger.info("Performing function interrupt")
    interrupt_event.set()


def init(node_name="CleverSwarmFlight", anon=True, no_signals=True):
    module_logger.info("Initing ROS node")
    rospy.init_node(node_name, anonymous=anon, disable_signals=no_signals)
    module_logger.info("Ros node inited")


def get_distance3d(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


def check(check_name):
    def inner(f):
        def wrapper(*args, **kwargs):
            result, failures = f(*args, **kwargs)
            if failures:
                msgs = []
                for failure in failures:
                    msg = "[{}]: Failure: {}".format(check_name, failure)
                    msgs.append(msg)
                    module_logger.warning(msg)
                return msgs
            else:
                module_logger.info("[{}]: OK".format(check_name))
                return None
        return wrapper
    return inner


@check("Linear velocity estimation")
def check_linear_speeds():
    failures = []
    telemetry = get_telemetry(frame_id='body')
    speed_limit = 0.1
    if telemetry.vx >= speed_limit:
        failures.append("X velocity estimation: {:.3f} m/s".format(telemetry.vx))
    if telemetry.vy >= speed_limit:
        failures.append("Y velocity estimation: {:.3f} m/s".format(telemetry.vy))
    if telemetry.vz >= speed_limit:
        failures.append("Z velocity estimation: {:.3f} m/s".format(telemetry.vz))
    return failures


@check("Angular velocity estimation")
def check_angular_speeds():
    failures = []
    telemetry = get_telemetry(frame_id='body')
    rate_limit = 0.05
    if telemetry.pitch_rate >= rate_limit:
        failures.append("Pitch rate estimation: {:.3f} rad/s".format(telemetry.pitch_rate))
    if telemetry.roll_rate >= rate_limit:
        failures.append("Roll rate estimation: {:.3f} rad/s".format(telemetry.roll_rate))
    if telemetry.yaw_rate >= rate_limit:
        failures.append("Yaw rate estimation: {:.3f} rad/s".format(telemetry.yaw_rate))
    return failures


@check("Angles estimation")
def check_angles():
    failures = []
    telemetry = get_telemetry(frame_id='body')
    angle_limit = math.radians(1)
    if abs(telemetry.pitch) >= angle_limit:
        failures.append("Pitch estimation: {:.3f} rad;{:.3f} degrees".format(telemetry.pitch,
                                                                             math.degrees(telemetry.pitch)))
    if abs(telemetry.roll) >= angle_limit:
        failures.append("Roll estimation: {:.3f} rad;{:.3f} degrees".format(telemetry.roll,
                                                                             math.degrees(telemetry.roll)))
    if abs(telemetry.yaw) >= angle_limit:
        failures.append("Yaw estimation: {:.3f} rad;{:.3f} degrees".format(telemetry.yaw,
                                                                             math.degrees(telemetry.yaw)))
    return failures


def selfcheck():
    msgs = []
    msgs.extend(check_linear_speeds())
    msgs.extend(check_angular_speeds())
    msgs.extend(check_angles())

    return msgs


def navto(x, y, z, yaw=float('nan'), frame_id='aruco_map'):
    set_position(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw)
    telemetry = get_telemetry(frame_id=frame_id)

    module_logger.info('Going to: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    module_logger.info('Telemetry now: | z: {:.3f}'.format(telemetry.z))

    return True


def reach_point(x=0.0, y=0.0, z=0.0, yaw=float('nan'), speed=1.0, tolerance=TOLERANCE, frame_id='aruco_map',
          freq=FREQUENCY, timeout=5000, wait=False):

    module_logger.info('Reaching point: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    navigate(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw, speed=speed)

    # waiting for completion
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = rospy.get_rostime()

    while (get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z) > tolerance) or wait:
        if interrupt_event.is_set():
            module_logger.warning("Flight function interrupted!")
            interrupt_event.clear()
            break

        telemetry = get_telemetry(frame_id=frame_id)
        module_logger.info('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
            telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))

        time_passed = (rospy.get_rostime() - time_start).to_sec() * 1000

        if timeout is not None:
            if time_passed >= timeout:
                module_logger.warning('Reaching point timed out! | time: {:3f} seconds'.format(time_passed / 1000))
                return wait
        rate.sleep()
    else:
        module_logger.info("Point reached!")
        return True


def reach_attitude(z=0.0, yaw=float('nan'), speed=1.0, tolerance=TOLERANCE, frame_id='aruco_map',
          freq=FREQUENCY, timeout=5000, wait=False):

    module_logger.info('Reaching attitude: | z: {:.3f} yaw: {:.3f}'.format(z, yaw))
    current_telem = get_telemetry(frame_id=frame_id)
    navigate(frame_id=frame_id, x=current_telem.x, y=current_telem.xy, z=z, yaw=yaw, speed=speed)

    # waiting for completion
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = rospy.get_rostime()

    while (abs(z - telemetry.z) > tolerance) or wait:
        if interrupt_event.is_set():
            module_logger.warning("Flight function interrupted!")
            interrupt_event.clear()
            break

        telemetry = get_telemetry(frame_id=frame_id)
        module_logger.info('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
            telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))

        time_passed = (rospy.get_rostime() - time_start).to_sec() * 1000
        if timeout is not None:
            if time_passed >= timeout:
                module_logger.warning('Reaching attitude timed out! | time: {:3f} seconds'.format(time_passed / 1000))
                return wait
        else:
            return True
        rate.sleep()
    else:
        module_logger.info("Attitude reached!")
        return True


def land(descend=True, z=1.0, frame_id_descend="aruco_map", frame_id_land="aruco_map",
         timeout_descend=5000, timeout_land=7500, freq=FREQUENCY):
    if descend:
        module_logger.info("Descending to: | z: {:.3f}".format(z))
        reach_attitude(z=z, frame_id=frame_id_descend, timeout=timeout_descend, freq=freq)
    landing()

    telemetry = get_telemetry(frame_id=frame_id_land)
    rate = rospy.Rate(1000 / wait_ms)
    time_start = rospy.get_rostime()

    while telemetry.armed:
        if interrupt_event.is_set():
            module_logger.warning("Flight function interrupted!")
            interrupt_event.clear()
            break

        telemetry = get_telemetry(frame_id=frame_id_land)
        module_logger.info("Landing...")
        time_passed = (rospy.get_rostime() - time_start).to_sec() * 1000

        if timeout_land is not None:
            if time_passed >= timeout_land:
                module_logger.warning('Landing timed out! | time: {:3f} seconds'.format(time_passed / 1000))
                module_logger.warning("Disarming!")
                arming(False)
                return False
        rate.sleep()
    else:
        module_logger.info("Landing succeeded!")
        return True


def takeoff(z=1.0,  speed=0.5, frame_id='body', freq=FREQUENCY,
            timeout_arm=750,  timeout_takeoff=5000, wait=False, emergency_land=True):
    module_logger.info("Starting takeoff!")
    module_logger.info("Arming, going to OFFBOARD mode")
    navigate(frame_id=frame_id, speed=speed_takeoff, auto_arm=True)

    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = rospy.get_rostime()

    while (not telemetry.armed) or wait:
        if interrupt_event.is_set():
            module_logger.warning("Flight function interrupted!")
            interrupt_event.clear()
            return None

        telemetry = get_telemetry(frame_id=frame_id_takeoff)
        module_logger.info("Arming...")
        time_passed = (rospy.get_rostime() - time_start).to_sec() * 1000

        if timeout is not None:
            if time_passed >= timeout_arm:
                module_logger.warning('Arming timed out! | time: {:3f} seconds'.format(time_passed / 1000))
                return False
        rate.sleep()

    module_logger.info("Armed!")

    result = reach_point(z=z, speed=speed, frame_id=frame_id, timeout=timeout_takeoff, freq=freq, wait=wait)
    if result:
        module_logger.info("Takeoff succeeded!")
    else:
        module_logger.warning("Takeoff navigation failed")
        if emergency_land:
            module_logger.info("Preforming emergency land")
            land(descend=False)

    return result

