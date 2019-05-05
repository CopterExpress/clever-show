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

logger = logging.getLogger(__name__)

# create proxy service
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_rates = rospy.ServiceProxy('/set_rates', srv.SetRates)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
landing = rospy.ServiceProxy('/land', Trigger)

logger.info("Proxy services inited")

# globals
FREQUENCY = 1000 / 25  # HZ
TOLERANCE = 0.2

interrupt_event = threading.Event()

checklist = []


def interrupt():
    logger.info("Performing function interrupt")
    interrupt_event.set()


def init(node_name="CleverSwarmFlight", anon=True, no_signals=True):
    logger.info("Initing ROS node")
    rospy.init_node(node_name, anonymous=anon, disable_signals=no_signals)
    logger.info("Ros node inited")


def get_distance3d(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


def check(check_name):
    def inner(f):
        def wrapper(*args, **kwargs):
            failures = f(*args, **kwargs)
            if failures:
                msgs = []
                for failure in failures:
                    msg = "[{}]: Failure: {}".format(check_name, failure)
                    msgs.append(msg)
                    logger.warning(msg)
                return msgs
            else:
                logger.info("[{}]: OK".format(check_name))
                return None

        checklist.append(wrapper)
        return wrapper

    return inner


@check("Linear velocity estimation")
def check_linear_speeds(speed_limit=0.1):
    telemetry = get_telemetry(frame_id='body')

    if telemetry.vx >= speed_limit:
        yield ("X velocity estimation: {:.3f} m/s".format(telemetry.vx))
    if telemetry.vy >= speed_limit:
        yield ("Y velocity estimation: {:.3f} m/s".format(telemetry.vy))
    if telemetry.vz >= speed_limit:
        yield ("Z velocity estimation: {:.3f} m/s".format(telemetry.vz))


@check("Angular velocity estimation")
def check_angular_speeds(rate_limit=0.05):
    telemetry = get_telemetry(frame_id='body')

    if telemetry.pitch_rate >= rate_limit:
        yield ("Pitch rate estimation: {:.3f} rad/s".format(telemetry.pitch_rate))
    if telemetry.roll_rate >= rate_limit:
        yield ("Roll rate estimation: {:.3f} rad/s".format(telemetry.roll_rate))
    if telemetry.yaw_rate >= rate_limit:
        yield ("Yaw rate estimation: {:.3f} rad/s".format(telemetry.yaw_rate))


@check("Angles estimation")
def check_angles(angle_limit=math.radians(5)):
    telemetry = get_telemetry(frame_id='body')
    if abs(telemetry.pitch) >= angle_limit:
        yield ("Pitch estimation: {:.3f} rad;{:.3f} degrees".format(telemetry.pitch,
                                                                    math.degrees(telemetry.pitch)))
    if abs(telemetry.roll) >= angle_limit:
        yield ("Roll estimation: {:.3f} rad;{:.3f} degrees".format(telemetry.roll,
                                                                   math.degrees(telemetry.roll)))
    if abs(telemetry.yaw) >= angle_limit:
        yield ("Yaw estimation: {:.3f} rad;{:.3f} degrees".format(telemetry.yaw,
                                                                  math.degrees(telemetry.yaw)))


def selfcheck():
    checks = []
    for check_f in checklist:
        msg = check_f()
        checks += msg if msg else []

    return checks


def navto(x, y, z, yaw=float('nan'), frame_id='aruco_map', **kwargs):
    set_position(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw)
    telemetry = get_telemetry(frame_id=frame_id)

    logger.info('Going to: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    logger.info('Telemetry now: | z: {:.3f}'.format(telemetry.z))

    return True


def reach_point(x=0.0, y=0.0, z=0.0, yaw=float('nan'), speed=1.0, tolerance=TOLERANCE, frame_id='aruco_map',
                freq=FREQUENCY, timeout=5000, wait=False, interrupter=interrupt_event):
    logger.info('Reaching point: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    navigate(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw, speed=speed)

    # waiting for completion
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = rospy.get_rostime()

    while (get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z) > tolerance) or wait:
        if interrupter.is_set():
            logger.warning("Flight function interrupted!")
            interrupter.clear()
            return

        telemetry = get_telemetry(frame_id=frame_id)
        logger.info('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
            telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))
        logger.info('Current delta: | {:.3f}'.format(
            get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z)))

        time_passed = (rospy.get_rostime() - time_start).to_sec() * 1000

        if timeout is not None:
            if time_passed >= timeout:
                logger.warning('Reaching point timed out! | time: {:3f} seconds'.format(time_passed / 1000))
                return wait
        rate.sleep()
    else:
        logger.info("Point reached!")
        return True


def reach_attitude(z=0.0, yaw=float('nan'), speed=1.0, tolerance=TOLERANCE, frame_id='aruco_map',
                   freq=FREQUENCY, timeout=5000, wait=False, interrupter=interrupt_event):
    logger.info('Reaching attitude: | z: {:.3f} yaw: {:.3f}'.format(z, yaw))
    current_telem = get_telemetry(frame_id=frame_id)
    navigate(frame_id=frame_id, x=current_telem.x, y=current_telem.y, z=z, yaw=yaw, speed=speed)

    # waiting for completion
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = rospy.get_rostime()

    while (abs(z - telemetry.z) > tolerance) or wait:
        if interrupter.is_set():
            logger.warning("Flight function interrupted!")
            interrupter.clear()
            return

        telemetry = get_telemetry(frame_id=frame_id)
        logger.info('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
            telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))

        time_passed = (rospy.get_rostime() - time_start).to_sec() * 1000
        if timeout is not None:
            if time_passed >= timeout:
                logger.warning('Reaching attitude timed out! | time: {:3f} seconds'.format(time_passed / 1000))
                return wait
        rate.sleep()
    else:
        logger.info("Attitude reached!")
        return True


def stop(frame_id='body', hold_speed=1.0):
    navigate(frame_id=frame_id, yaw=float('nan'), speed=hold_speed)


def land(descend=True, z=1.0, frame_id_descend="aruco_map", frame_id_land="aruco_map",
         timeout_descend=5000, timeout_land=7500, freq=FREQUENCY, interrupter=interrupt_event):
    if descend:
        logger.info("Descending to: | z: {:.3f}".format(z))
        reach_attitude(z=z, frame_id=frame_id_descend, timeout=timeout_descend, freq=freq, yaw=1.57,  # TODO yaw
                       interrupter=interrupter)
    landing()

    telemetry = get_telemetry(frame_id='aruco_map')
    rate = rospy.Rate(freq)
    time_start = rospy.get_rostime()

    while telemetry.armed:
        if interrupter.is_set():
            logger.warning("Flight function interrupted!")
            interrupter.clear()
            return

        telemetry = get_telemetry(frame_id=frame_id_land)
        logger.info("Landing... | z: {}".format(telemetry.z))
        time_passed = (rospy.get_rostime() - time_start).to_sec() * 1000

        if timeout_land is not None:
            if time_passed >= timeout_land:
                logger.warning('Landing timed out! | time: {:3f} seconds'.format(time_passed / 1000))
                logger.warning("Disarming!")
                arming(False)
                return False
        rate.sleep()
    else:
        logger.info("Landing succeeded!")
        return True


def takeoff(z=1.0, speed=0.8, frame_id='body', freq=FREQUENCY,
            timeout_arm=2000, timeout_takeoff=5000, wait=False, tolerance=TOLERANCE, emergency_land=False,
            interrupter=interrupt_event):
    logger.info("Starting takeoff!")
    logger.info("Arming, going to OFFBOARD mode")

    # Arming check
    set_rates(thrust=0.1, auto_arm=True)
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = rospy.get_rostime()

    while (not telemetry.armed) or wait:
        if interrupter.is_set():
            logger.warning("Flight function interrupted!")
            interrupter.clear()
            return

        telemetry = get_telemetry(frame_id=frame_id)
        logger.info("Arming...")
        time_passed = (rospy.get_rostime() - time_start).to_sec() * 1000

        if timeout_arm is not None:
            if time_passed >= timeout_arm:
                if not telemetry.armed:
                    logger.warning('Arming timed out! | time: {:3f} seconds'.format(time_passed / 1000))
                    return False
                else:
                    break
        rate.sleep()

    logger.info("Armed!")

    # Reach height
    z0 = get_telemetry().z
    z_dest = z + z0
    navigate(z=z_dest, speed=speed, frame_id=frame_id, auto_arm=True)
    current_height = abs(get_telemetry().z - z_dest)
    while current_height > tolerance or wait:
        if interrupter.is_set():
            logger.warning("Flight function interrupted!")
            interrupter.clear()
            return

        current_height = abs(get_telemetry().z - z_dest)
        logger.info("Takeoff...")
        time_passed = (rospy.get_rostime() - time_start).to_sec() * 1000

        if timeout_takeoff is not None:
            if time_passed >= timeout_takeoff:
                if not wait:
                    logger.warning('Takeoff timed out! | time: {:3f} seconds'.format(time_passed / 1000))
                    if emergency_land:
                        logger.info("Preforming emergency land")
                        land(descend=False, interrupter=interrupter)
                    return False
                else:
                    break
        rate.sleep()

    logger.info("Takeoff succeeded!")
    # print("Takeoff succeeded!")
    return True
