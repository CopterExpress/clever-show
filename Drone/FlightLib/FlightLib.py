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
FREQUENCY = 40  # HZ
TOLERANCE = 0.2
SPEED = 1.0
SPEED_TAKEOFF = 0.8
TIMEOUT = 5.0
TIMEOUT_ARMED = 2.0
TIMEOUT_DESCEND = TIMEOUT
TIMEOUT_LAND = 8.0
Z_DESCEND = 0.5
Z_TAKEOFF = 1.0
FRAME_ID = 'map'
INTERRUPTER = threading.Event()

checklist = []

def arming_wrapper(state=False, interrupter=INTERRUPTER):
    arming(state)

def interrupt():
    logger.info("Performing function interrupt")
    INTERRUPTER.set()


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
            print(failures)
            msgs = []
            for failure in failures:
                print(failure)
                msg = "[{}]: Failure: {}".format(check_name, failure)
                msgs.append(msg)
                logger.warning(msg)

            if msgs:
                return msgs
            else:
                logger.info("[{}]: OK".format(check_name))
                return None

        checklist.append(wrapper)
        return wrapper

    return inner


def _check_nans(*values):
    return any(math.isnan(x) for x in values)


@check("FCU connection")
def check_connection():
    telemetry = get_telemetry()
    if not telemetry.connected:
        yield ("Flight controller is not connected!")


@check("Linear velocity estimation")
def check_linear_speeds(speed_limit=0.1):
    telemetry = get_telemetry(frame_id='body')

    if _check_nans(telemetry.vx, telemetry.vy, telemetry.vz):
        yield ("Velocity estimation is not available")

    if telemetry.vx >= speed_limit:
        yield ("X velocity estimation: {:.3f} m/s".format(telemetry.vx))
    if telemetry.vy >= speed_limit:
        yield ("Y velocity estimation: {:.3f} m/s".format(telemetry.vy))
    if telemetry.vz >= speed_limit:
        yield ("Z velocity estimation: {:.3f} m/s".format(telemetry.vz))


@check("Angular velocity estimation")
def check_angular_speeds(rate_limit=0.05):
    telemetry = get_telemetry(frame_id='body')

    if _check_nans(telemetry.pitch_rate, telemetry.roll_rate, telemetry.yaw_rate):
        yield ("Angular velocities estimation is not available")

    if telemetry.pitch_rate >= rate_limit:
        yield ("Pitch rate estimation: {:.3f} rad/s".format(telemetry.pitch_rate))
    if telemetry.roll_rate >= rate_limit:
        yield ("Roll rate estimation: {:.3f} rad/s".format(telemetry.roll_rate))
    if telemetry.yaw_rate >= rate_limit:
        yield ("Yaw rate estimation: {:.3f} rad/s".format(telemetry.yaw_rate))


@check("Angles estimation")
def check_angles(angle_limit=math.radians(5)):
    telemetry = get_telemetry(frame_id='body')

    if _check_nans(telemetry.pitch, telemetry.roll, telemetry.yaw):
        yield ("Angular velocities estimation is not available")

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


def navto(x, y, z, yaw=float('nan'), frame_id=FRAME_ID, **kwargs):
    set_position(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw)
    telemetry = get_telemetry(frame_id=frame_id)

    logger.info('Going to: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    print('Going to: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    logger.info('Telemetry now: | z: {:.3f}'.format(telemetry.z))
    print('Telemetry now: | z: {:.3f}'.format(telemetry.z))

    return True


def reach_point(x=0.0, y=0.0, z=0.0, yaw=float('nan'), speed=SPEED, tolerance=TOLERANCE, frame_id=FRAME_ID,
                freq=FREQUENCY, timeout=TIMEOUT, interrupter=INTERRUPTER, wait=False):
    logger.info('Reaching point: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    print('Reaching point: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))
    navigate(frame_id=frame_id, x=x, y=y, z=z, yaw=yaw, speed=speed)

    # waiting for completion
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = time.time()

    while (get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z) > tolerance) or wait:
        if interrupter.is_set():
            logger.warning("Reach point function interrupted!")
            print("Reach point function interrupted!")
            interrupter.clear()
            return False

        telemetry = get_telemetry(frame_id=frame_id)
        logger.info('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
            telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))
        print('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
            telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))
        logger.info('Current delta: | {:.3f}'.format(
            get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z)))
        print('Current delta: | {:.3f}'.format(
            get_distance3d(x, y, z, telemetry.x, telemetry.y, telemetry.z)))

        time_passed = time.time() - time_start

        if timeout is not None:
            if time_passed >= timeout:
                logger.warning('Reaching point timed out! | time: {:3f} seconds'.format(time_passed))
                print('Reaching point timed out! | time: {:3f} seconds'.format(time_passed))
                return wait
        rate.sleep()

    logger.info("Point reached!")
    print("Point reached!")
    return True


def reach_altitude(z=0.0, yaw=float('nan'), speed=SPEED, tolerance=TOLERANCE, frame_id=FRAME_ID,
                   freq=FREQUENCY, timeout=TIMEOUT, interrupter=INTERRUPTER, wait=False):
    logger.info('Reaching attitude: | z: {:.3f} yaw: {:.3f}'.format(z, yaw))
    print('Reaching attitude: | z: {:.3f} yaw: {:.3f}'.format(z, yaw))
    current_telem = get_telemetry(frame_id=frame_id)
    navigate(frame_id=frame_id, x=current_telem.x, y=current_telem.y, z=z, yaw=yaw, speed=speed)

    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = time.time()

    while (abs(z - telemetry.z) > tolerance) or wait:
        if interrupter.is_set():
            logger.warning("Reach altitude function interrupted!")
            print("Reach altitude function interrupted!")
            interrupter.clear()
            return

        telemetry = get_telemetry(frame_id=frame_id)
        logger.info('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
            telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))
        print('Telemetry: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(
            telemetry.x, telemetry.y, telemetry.z, telemetry.yaw))

        time_passed = time.time() - time_start
        if timeout is not None:
            if time_passed >= timeout:
                logger.warning('Reaching attitude timed out! | time: {:3f} seconds'.format(time_passed))
                print('Reaching attitude timed out! | time: {:3f} seconds'.format(time_passed))
                return wait
        rate.sleep()

    logger.info("Altitude reached!")
    print("Altitude reached!")
    return True


def stop(frame_id='body', hold_speed=SPEED):
    navigate(frame_id=frame_id, yaw=float('nan'), speed=hold_speed)


def land(descend=True, z=Z_DESCEND, frame_id_descend=FRAME_ID, frame_id_land=FRAME_ID,
         timeout_descend=TIMEOUT_DESCEND, timeout_land=TIMEOUT_LAND, freq=FREQUENCY, interrupter=INTERRUPTER):
    if descend:
        logger.info("Descending to: | z: {:.3f}".format(z))
        print("Descending to: | z: {:.3f}".format(z))
        reach_altitude(z=z, frame_id=frame_id_descend, timeout=timeout_descend, freq=freq, yaw=float('nan'),  # TODO yaw
                       interrupter=interrupter)
    landing()
    print("Land is started")

    telemetry = get_telemetry(frame_id=frame_id_land)
    rate = rospy.Rate(freq)
    time_start = time.time()

    while telemetry.armed:
        if interrupter.is_set():
            logger.warning("Land function interrupted!")
            print("Land function interrupted!")
            interrupter.clear()
            return False

        telemetry = get_telemetry(frame_id=frame_id_land)
        logger.info("Landing... | z: {}".format(telemetry.z))
        print("Landing... | z: {}".format(telemetry.z))
        time_passed = time.time() - time_start

        if timeout_land is not None:
            if time_passed >= timeout_land:
                logger.warning('Landing timed out! | time: {:3f} seconds'.format(time_passed))
                logger.warning("Disarming!")
                print("Landing timed out, disarming!!!")
                arming(False)
                return False
        rate.sleep()

    logger.info("Landing succeeded!")
    print("Landing succeeded!")
    return True


def takeoff(z=Z_TAKEOFF, speed=SPEED_TAKEOFF, frame_id='body', freq=FREQUENCY,
            timeout_arm=TIMEOUT_ARMED, timeout_takeoff=TIMEOUT, wait=False, tolerance=TOLERANCE, emergency_land=False,
            interrupter=INTERRUPTER):
    logger.info("Starting takeoff!")
    print("Starting takeoff!")
    logger.info("Arming, going to OFFBOARD mode")

    # Arming check
    set_rates(thrust=0.1, auto_arm=True)
    telemetry = get_telemetry(frame_id=frame_id)
    rate = rospy.Rate(freq)
    time_start = time.time()

    while (not telemetry.armed) or wait:
        if interrupter.is_set():
            logger.warning("Takeoff function interrupted!")
            print("Takeoff function interrupted!")
            interrupter.clear()
            return

        telemetry = get_telemetry(frame_id=frame_id)
        logger.info("Arming...")
        print("Arming...")
        time_passed = time.time() - time_start

        if timeout_arm is not None:
            if time_passed >= timeout_arm:
                if not telemetry.armed:
                    logger.warning('Arming timed out! | time: {:3f} seconds'.format(time_passed))
                    print('Arming timed out! | time: {:3f} seconds'.format(time_passed))
                    return False
                else:
                    break
        rate.sleep()

    logger.info("Armed!")
    print("Armed")

    # Reach height
    z0 = get_telemetry().z
    z_dest = z + z0
    navigate(z=z, speed=speed, frame_id=frame_id, auto_arm=True)
    current_diff = abs(get_telemetry().z - z_dest)
    while (current_diff > tolerance) or wait:
        if interrupter.is_set():
            logger.warning("Flight function interrupted!")
            print("Flight function interrupted!")
            interrupter.clear()
            return

        current_diff = abs(get_telemetry().z - z_dest)
        logger.info("Takeoff...")
        print("Takeoff...")
        time_passed = time.time() - time_start

        if timeout_takeoff is not None:
            if time_passed >= timeout_takeoff:
                if not wait:
                    logger.warning('Takeoff timed out! | time: {:3f} seconds'.format(time_passed))
                    print('Takeoff timed out! | time: {:3f} seconds'.format(time_passed))
                    if emergency_land:
                        logger.info("Preforming emergency land")
                        print("Preforming emergency land")
                        land(descend=False, interrupter=interrupter)
                    return False
                else:
                    break
        rate.sleep()

    logger.info("Takeoff succeeded!")
    print("Takeoff succeeded!")
    return True
