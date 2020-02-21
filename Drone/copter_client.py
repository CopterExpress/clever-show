import os
import sys
import time
import math
import rospy
import numpy
from clever import srv
import datetime
import logging
import threading
import psutil
import subprocess
from collections import namedtuple

from FlightLib import FlightLib
from FlightLib import LedLib

import client

import messaging_lib as messaging
import tasking_lib as tasking
import animation_lib as animation

from mavros_mavlink import *

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import tf2_ros

static_bloadcaster = tf2_ros.StaticTransformBroadcaster()

emergency = False

logging.basicConfig(  # TODO all prints as logs
    level=logging.DEBUG,  # INFO
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

animation_logger = logging.getLogger('animation_lib')
animation_logger.setLevel(logging.INFO)
animation_logger.addHandler(handler)

client_logger = logging.getLogger('client')
client_logger.setLevel(logging.DEBUG)
client_logger.addHandler(handler)

messaging_logger = logging.getLogger('messaging_lib')
messaging_logger.setLevel(logging.INFO)
messaging_logger.addHandler(handler)

tasking_logger = logging.getLogger('tasking_lib')
tasking_logger.setLevel(logging.INFO)
tasking_logger.addHandler(handler)

flightlib_logger = logging.getLogger('FlightLib')
flightlib_logger.setLevel(logging.INFO)
flightlib_logger.addHandler(handler)


class CopterClient(client.Client):
    def __init__(self, config_path="config/client.ini"):
        super(CopterClient, self).__init__(config_path)
        self.load_config()
        self.frames = {}

    def load_config(self):
        super(CopterClient, self).load_config()

    def on_broadcast_bind(self):
        repair_chrony(self.config.server_host)

    def start(self, task_manager_instance):
        rospy.loginfo("Init ROS node")
        rospy.init_node('clever_show_client')
        if self.config.led_use:
            LedLib.init_led(self.config.led_pin)
        task_manager_instance.start()  # TODO move to self
        if self.config.copter_frame_id == "floor":
            if self.config.floor_frame_enabled:
                self.start_floor_frame_broadcast()
            else:
                rospy.logerr("Can't make floor frame!")
        start_subscriber()

        telemetry.start_loop()
        super(CopterClient, self).start()

    def start_floor_frame_broadcast(self):
        trans = TransformStamped()
        trans.transform.translation.x = self.config.floor_frame_translation[0]
        trans.transform.translation.y = self.config.floor_frame_translation[1]
        trans.transform.translation.z = self.config.floor_frame_translation[2]
        trans.transform.rotation = Quaternion(*quaternion_from_euler(math.radians(self.config.floor_frame_rotation[0]),
                                                                     math.radians(self.config.floor_frame_rotation[1]),
                                                                     math.radians(self.config.floor_frame_rotation[2])))
        trans.header.frame_id = self.config.floor_frame_parent
        trans.child_frame_id = self.config.copter_frame_id
        static_bloadcaster.sendTransform(trans)


def restart_service(name):
    os.system("systemctl restart {}".format(name))

def repair_chrony(ip):
    logger.info("Configure chrony ip to {}".format(ip))
    configure_chrony_ip(ip)
    restart_service("chrony")

def execute_command(command):
    os.system(command)


def configure_chrony_ip(ip, path="/etc/chrony/chrony.conf", ip_index=1):  # TODO simplify
    try:
        with open(path, 'r') as f:
            raw_content = f.read()
    except IOError as e:
        logger.error("Reading error {}".format(e))
        return False

    content = raw_content.split(" ")

    try:
        current_ip = content[ip_index]
    except IndexError:
        logger.error("Something wrong with config")
        return False

    if "." not in current_ip:
        logger.debug("That's not ip!")

    if current_ip != ip:
        content[ip_index] = ip

        try:
            with open(path, 'w') as f:
                f.write(" ".join(content))
        except IOError:
            logger.error("Error writing")
            return False

    return True


def configure_hostname(hostname):
    path = "/etc/hostname"
    try:
        with open(path, 'r') as f:
            raw_content = f.read()
    except IOError as e:
        logger.error("Reading error {}".format(e))
        return False

    current_hostname = str(raw_content)

    if current_hostname != hostname:
        content = hostname + '\n'
        try:
            with open(path, 'w') as f:
                f.write(content)
        except IOError:
            logger.error("Error writing")
            return False

    return True


def configure_hosts(hostname):
    path = "/etc/hosts"
    try:
        with open(path, 'r') as f:
            raw_content = f.read()
    except IOError as e:
        logger.error("Reading error {}".format(e))
        return False

    index_start = raw_content.find("127.0.1.1", )
    index_stop = raw_content.find("\n", index_start)

    hosts_array = raw_content[index_start:index_stop].split()
    _ip = hosts_array[0]
    current_hostname = hosts_array[1]
    if current_hostname != hostname:
        content = raw_content[:index_start] + "{}       {} {}.local".format(_ip, hostname, hostname) + raw_content[
                                                                                                       index_stop:]
        try:
            with open(path, 'w') as f:
                f.write(content)
        except IOError:
            logger.error("Error writing")
            return False

    return True


def configure_motd(hostname):
    with open("/etc/motd", "w") as f:
        f.write("\r\n{}\r\n\r\n".format(hostname))


def configure_bashrc(hostname):
    path = "/home/pi/.bashrc"
    try:
        with open(path, 'r') as f:
            raw_content = f.read()
    except IOError as e:
        logger.error("Reading error {}".format(e))
        return False

    index_start = raw_content.find("ROS_HOSTNAME='", ) + 14
    index_stop = raw_content.find("'", index_start)

    current_hostname = raw_content[index_start:index_stop]
    if current_hostname != hostname:
        content = raw_content[:index_start] + hostname + raw_content[index_stop:]
        try:
            with open(path, 'w') as f:
                f.write(content)
        except IOError:
            logger.error("Error writing")
            return False

    return True


@messaging.message_callback("execute")
def _execute(*args, **kwargs):
    command = kwargs.get("command", None)
    if command is not None:
        logger.info("Executing command: {}".format(command))
        execute_command(command)
        logger.info("Executing done")


@messaging.message_callback("id")  # TODO redo
def _response_id(*args, **kwargs):
    new_id = kwargs.get("new_id", None)
    if new_id is not None:
        old_id = client.active_client.client_id
        if new_id != old_id:
            client.active_client.config.set('PRIVATE', 'id', new_id, write=True)
            client.active_client.client_id = new_id
            if new_id != '/hostname':
                if client.active_client.config.system_restart_after_rename:
                    hostname = client.active_client.client_id
                    configure_hostname(hostname)
                    configure_hosts(hostname)
                    configure_bashrc(hostname)
                    configure_motd(hostname)
                    execute_command("systemctl stop clever-show & reboot")
                    # execute_command("hostname {}".format(hostname))
                    # restart_service("dhcpcd")
                    # restart_service("avahi-daemon")
                    # restart_service("smbd")
                    # restart_service("roscore")
                    # restart_service("clever")
                restart_service("clever-show")


@messaging.request_callback("selfcheck")
def _response_selfcheck(*args, **kwargs):
    if check_state_topic(wait_new_status=True):
        check = FlightLib.selfcheck()
        return check if check else "OK"
    else:
        stop_subscriber()
        return "NOT_CONNECTED_TO_FCU"


@messaging.request_callback("telemetry")
def _response_telemetry(*args, **kwargs):
    telemetry.update()
    return telemetry.create_msg_contents()


@messaging.request_callback("anim_id")
def _response_animation_id(*args, **kwargs):
    # Load animation
    result = animation.get_id()
    if result != 'No animation':
        logger.debug("Saving corrected animation")
        offset = numpy.array(client.active_client.config.private_offset) + numpy.array(client.active_client.config.copter_common_offset)
        frames = animation.load_animation(os.path.abspath("animation.csv"), client.active_client.config.animation_frame_delay, 
                                            offset[0], offset[1], offset[2], *client.active_client.config.animation_ratio)
        # Correct start and land frames in animation
        corrected_frames, start_action, start_delay = animation.correct_animation(frames,
                                                        check_takeoff=client.active_client.config.animation_takeoff_detection,
                                                        check_land=client.active_client.config.animation_land_detection,
                                                        )
        logger.debug("Start action: {}".format(start_action))
        # Save corrected animation
        animation.save_corrected_animation(corrected_frames)
    return result


@messaging.request_callback("batt_voltage")
def _response_batt(*args, **kwargs):
    if check_state_topic(wait_new_status=True):
        return FlightLib.get_telemetry_locked('body').voltage
    else:
        stop_subscriber()
        return float('nan')


@messaging.request_callback("cell_voltage")
def _response_cell(*args, **kwargs):
    if check_state_topic(wait_new_status=True):
        return FlightLib.get_telemetry_locked('body').cell_voltage
    else:
        stop_subscriber()
        return float('nan')


@messaging.request_callback("sys_status")
def _response_sys_status(*args, **kwargs):
    return get_sys_status()


@messaging.request_callback("cal_status")
def _response_cal_status(*args, **kwargs):
    if check_state_topic(wait_new_status=True):
        return get_calibration_status()
    else:
        stop_subscriber()
        return "NOT_CONNECTED_TO_FCU"


@messaging.request_callback("position")
def _response_position(*args, **kwargs):
    telem = FlightLib.get_telemetry_locked(client.active_client.config.copter_frame_id)
    return "{:.2f} {:.2f} {:.2f} {:.1f} {}".format(
        telem.x, telem.y, telem.z, math.degrees(telem.yaw), client.active_client.config.copter_frame_id)


@messaging.request_callback("calibrate_gyro")
def _calibrate_gyro(*args, **kwargs):
    calibrate('gyro')
    return get_calibration_status()


@messaging.request_callback("calibrate_level")
def _calibrate_level(*args, **kwargs):
    calibrate('level')
    return get_calibration_status()


@messaging.request_callback("load_params")
def _load_params(*args, **kwargs):
    result = load_param_file('temp.params')
    logger.info("Load parameters to FCU success: {}".format(result))
    return result


@messaging.message_callback("test")
def _command_test(*args, **kwargs):
    logger.info("logging info test")
    rospy.logdebug("ros logdebug test")
    print("stdout test")


@messaging.message_callback("move_start")
def _command_move_start_to_current_position(*args, **kwargs):
    x_start, y_start = animation.get_start_xy(os.path.abspath("animation.csv"),
                                            *client.active_client.config.animation_ratio)
    logger.debug("x_start = {}, y_start = {}".format(x_start, y_start))
    if not math.isnan(x_start):
        telem = FlightLib.get_telemetry_locked(client.active_client.config.copter_frame_id)
        logger.debug("x_telem = {}, y_telem = {}".format(telem.x, telem.y))
        if not math.isnan(telem.x):
            client.active_client.config.set('PRIVATE', 'offset', 
                                            [telem.x - x_start, telem.y - y_start, client.active_client.config.private_offset[2]],
                                            write=True)
            logger.info("Set start delta: {:.2f} {:.2f}".format(client.active_client.config.private_offset[0], 
                                                                client.active_client.config.private_offset[1]))
        else:
            logger.debug("Wrong telemetry")
    else:
        logger.debug("Wrong animation file")


@messaging.message_callback("reset_start")
def _command_reset_start(*args, **kwargs):
    client.active_client.config.set('PRIVATE', 'offset', 
                                    [0, 0, client.active_client.config.private_offset[2]],
                                    write=True)
    logger.info("Reset start to {:.2f} {:.2f}".format(client.active_client.config.private_offset[0], 
                                                    client.active_client.config.private_offset[1]))


@messaging.message_callback("set_z_to_ground")
def _command_set_z(*args, **kwargs):
    telem = FlightLib.get_telemetry_locked(client.active_client.config.copter_frame_id)
    client.active_client.config.set('PRIVATE', 'offset', 
                                    [client.active_client.config.private_offset[0], client.active_client.config.private_offset[1], telem.z],
                                    write=True)
    logger.info("Set z offset to {:.2f}".format(client.active_client.config.private_offset[2]))


@messaging.message_callback("reset_z_offset")
def _command_reset_z(*args, **kwargs):
    client.active_client.config.set('PRIVATE', 'offset', 
                                    [client.active_client.config.private_offset[0], client.active_client.config.private_offset[1], 0],
                                    write=True)
    logger.info("Reset z offset to {:.2f}".format(client.active_client.config.private_offset[2]))


@messaging.message_callback("update_repo")
def _command_update_repo(*args, **kwargs):
    os.system("mv /home/pi/clever-show/Drone/client_config.ini /home/pi/clever-show/Drone/client_config_tmp.ini")
    os.system("git reset --hard HEAD")
    os.system("git checkout master")
    os.system("git fetch")
    os.system("git pull --rebase")
    os.system("mv /home/pi/clever-show/Drone/client_config_tmp.ini /home/pi/clever-show/Drone/client_config.ini")
    os.system("chown -R pi:pi /home/pi/clever-show")


@messaging.message_callback("reboot_all")
def _command_reboot_all(*args, **kwargs):
    reboot_fcu()
    execute_command("reboot")


@messaging.message_callback("reboot_fcu")
def _command_reboot(*args, **kwargs):
    reboot_fcu()


@messaging.message_callback("service_restart")
def _command_service_restart(*args, **kwargs):
    service = kwargs["name"]
    restart_service(service)


@messaging.message_callback("repair_chrony")
def _command_chrony_repair(*args, **kwargs):
    repair_chrony(client.active_client.config.server_host)


@messaging.message_callback("led_test")
def _command_led_test(*args, **kwargs):
    LedLib.chase(255, 255, 255)
    time.sleep(2)
    LedLib.off()


@messaging.message_callback("led_fill")
def _command_led_fill(*args, **kwargs):
    r = kwargs.get("red", 0)
    g = kwargs.get("green", 0)
    b = kwargs.get("blue", 0)

    LedLib.fill(r, g, b)


@messaging.message_callback("flip")
def _copter_flip(*args, **kwargs):
    FlightLib.flip(frame_id=client.active_client.config.copter_frame_id)


@messaging.message_callback("takeoff")
def _command_takeoff(*args, **kwargs):
    logger.info("Takeoff at {}".format(datetime.datetime.now()))
    task_manager.add_task(0, 0, animation.takeoff,
                          task_kwargs={
                              "z": client.active_client.config.copter_takeoff_height,
                              "timeout": client.active_client.config.copter_takeoff_time,
                              "safe_takeoff": client.active_client.config.copter_safe_takeoff,
                              "use_leds": client.active_client.config.led_use,
                          }
                          )


@messaging.message_callback("takeoff_z")
def _command_takeoff_z(*args, **kwargs):
    z_str = kwargs.get("z", None)
    if z_str is not None:
        telem = FlightLib.get_telemetry_locked(client.active_client.config.copter_frame_id)
        logger.info("Takeoff to z = {} at {}".format(z_str, datetime.datetime.now()))
        task_manager.add_task(0, 0, FlightLib.reach_point,
                              task_kwargs={
                                  "x": telem.x,
                                  "y": telem.y,
                                  "z": float(z_str),
                                  "frame_id": client.active_client.config.copter_frame_id,
                                  "timeout": client.active_client.config.copter_takeoff_time,
                                  "auto_arm": True,
                              }
                              )


@messaging.message_callback("land")
def _command_land(*args, **kwargs):
    task_manager.reset()
    task_manager.add_task(0, 0, animation.land,
                          task_kwargs={
                              "z": client.active_client.config.copter_takeoff_height,
                              "timeout": client.active_client.config.copter_takeoff_time,
                              "frame_id": client.active_client.config.copter_frame_id,
                              "use_leds": client.active_client.config.led_use,
                          }
                          )


@messaging.message_callback("emergency_land")
def _emergency_land(*args, **kwargs):
    logger.info(FlightLib.emergency_land().message)


@messaging.message_callback("disarm")
def _command_disarm(*args, **kwargs):
    task_manager.reset()
    task_manager.add_task(-5, 0, FlightLib.arming_wrapper,
                          task_kwargs={
                              "state": False
                          }
                          )


@messaging.message_callback("stop")
def _command_stop(*args, **kwargs):
    task_manager.reset()


@messaging.message_callback("pause")
def _command_pause(*args, **kwargs):
    task_manager.pause()


@messaging.message_callback("resume")
def _command_resume(*args, **kwargs):
    task_manager.resume(time_to_start_next_task=kwargs.get("time", 0))


@messaging.message_callback("start")
def _play_animation(*args, **kwargs):
    start_time = float(kwargs["time"])
    # Check if animation file is available
    if animation.get_id() == 'No animation':
        logger.error("Can't start animation without animation file!")
        return

    task_manager.reset(interrupt_next_task=False)

    logger.info("Start time = {}, wait for {} seconds".format(start_time, start_time - time.time()))
    # Load animation
    offset = numpy.array(client.active_client.config.private_offset) + numpy.array(client.active_client.config.copter_common_offset)
    frames = animation.load_animation(os.path.abspath("animation.csv"), client.active_client.config.animation_frame_delay, 
                                            offset[0], offset[1], offset[2], *client.active_client.config.animation_ratio)
    # Correct start and land frames in animation
    corrected_frames, start_action, start_delay = animation.correct_animation(frames,
                                                        check_takeoff=client.active_client.config.animation_takeoff_detection,
                                                        check_land=client.active_client.config.animation_land_detection,
                                                        )
    # Choose start action
    if start_action == 'takeoff':
        # Takeoff first
        task_manager.add_task(start_time, 0, animation.takeoff,
                              task_kwargs={
                                  "z": client.active_client.config.copter_takeoff_height,
                                  "timeout": client.active_client.config.copter_takeoff_time,
                                  "safe_takeoff": client.active_client.config.copter_safe_takeoff,
                                  # "frame_id": client.active_client.config.copter_frame_id,
                                  "use_leds": client.active_client.config.led_use,
                              }
                              )
        # Fly to first point
        rfp_time = start_time + client.active_client.config.copter_takeoff_time
        task_manager.add_task(rfp_time, 0, animation.execute_frame,
                              task_kwargs={
                                  "point": animation.convert_frame(corrected_frames[0])[0],
                                  "color": animation.convert_frame(corrected_frames[0])[1],
                                  "frame_id": client.active_client.config.copter_frame_id,
                                  "use_leds": client.active_client.config.led_use,
                                  "flight_func": FlightLib.reach_point,
                              }
                              )
        # Calculate first frame start time
        frame_time = rfp_time + client.active_client.config.copter_reach_first_point_time

    elif start_action == 'arm':
        # Calculate start time
        start_time += start_delay
        # Arm
        # task_manager.add_task(start_time, 0, FlightLib.arming_wrapper,
        #                    task_kwargs={
        #                        "state": True
        #                    }
        #                    )
        frame_time = start_time  # + 1.0
        point, color, yaw = animation.convert_frame(corrected_frames[0])
        task_manager.add_task(frame_time, 0, animation.execute_frame,
                              task_kwargs={
                                  "point": point,
                                  "color": color,
                                  "frame_id": client.active_client.config.copter_frame_id,
                                  "use_leds": client.active_client.config.led_use,
                                  "flight_func": FlightLib.navto,
                                  "auto_arm": True,
                              }
                              )
        # Calculate first frame start time
        frame_time += corrected_frames[0]["delay"]  # TODO Think about arming time
    logger.debug(task_manager.task_queue)
    # Play animation file
    for frame in corrected_frames:
        point, color, yaw = animation.convert_frame(frame)
        if client.active_client.config.animation_yaw == "animation":
            yaw = frame["yaw"]
        else:
            yaw = math.radians(float(client.active_client.config.animation_yaw))
        task_manager.add_task(frame_time, 0, animation.execute_frame,
                              task_kwargs={
                                  "point": point,
                                  "color": color,
                                  "yaw": yaw,
                                  "frame_id": client.active_client.config.copter_frame_id,
                                  "use_leds": client.active_client.config.led_use,
                                  "flight_func": FlightLib.navto,
                              }
                              )
        frame_time += frame["delay"]

    # Calculate land_time
    land_time = frame_time + client.active_client.config.copter_land_time
    # Land
    task_manager.add_task(land_time, 0, animation.land,
                          task_kwargs={
                              "timeout": client.active_client.config.copter_land_timeout,
                              "frame_id": client.active_client.config.copter_frame_id,
                              "use_leds": client.active_client.config.led_use,
                          },
                          )


# noinspection PyAttributeOutsideInit
class Telemetry:
    params_default_dict = {
        "git_version": None,
        "animation_id": None,
        "battery": None,
        "armed": False,
        "fcu_status": None,
        "calibration_status": None,
        "mode": None,
        "selfcheck": None,
        "current_position": None,
        "start_position": None,
        "last_task": None,
        "time_delta": None,
        "config_version": None,
    }

    def __init__(self):
        self._lock = threading.Lock()
        self._last_state = []
        self._interruption_counter = 0
        self._max_interruptions = 2
        self._tasks_cleared = False
        self.ros_telemetry = None

        for key, value in self.params_default_dict.items():
            setattr(self, key, value)

    def __setattr__(self, key, value):
        if key in self.params_default_dict:
            with self.__dict__['_lock']:
                self.__dict__[key] = value
        else:
            self.__dict__[key] = value

    def __getattr__(self, item):
        if item in self.params_default_dict:
            with self.__dict__['_lock']:
                return self.__dict__[item]

        return self.__dict__[item]

    @classmethod
    def get_git_version(cls):
        return subprocess.check_output("git log --pretty=format:'%h' -n 1", shell=True)

    @classmethod
    def get_config_version(cls):
        return "{} V{}".format(client.active_client.config.config_name, client.active_client.config.config_version)

    @classmethod
    def get_start_position(cls):
        x_start, y_start = animation.get_start_xy(os.path.abspath("animation.csv"),
                                                  *client.active_client.config.animation_ratio)
        offset = numpy.array(client.active_client.config.private_offset) + numpy.array(client.active_client.config.copter_common_offset)
        x = x_start + offset[0]
        y = y_start + offset[1]
        z = offset[2]
        if not FlightLib._check_nans(x, y, z):
            return x, y, z
        return 'NO_POS'

    @classmethod
    def get_battery(cls, ros_telemetry):
        if ros_telemetry is None:
            return float('nan'), float('nan')

        battery_v = ros_telemetry.voltage

        batt_empty_param = get_param('BAT_V_EMPTY')
        batt_charged_param = get_param('BAT_V_CHARGED')
        batt_cells_param = get_param('BAT_N_CELLS')

        if batt_empty_param.success and batt_charged_param.success and batt_cells_param.success:
            batt_empty = batt_empty_param.value.real
            batt_charged = batt_charged_param.value.real
            batt_cells = batt_cells_param.value.integer

            battery_p = (ros_telemetry.voltage / batt_cells - batt_empty) / (batt_charged - batt_empty) * 1.
            battery_p = max(min(battery_p, 1.), 0.)
        else:
            battery_p = float('nan')

        return battery_v, battery_p

    @classmethod
    def get_selfcheck(cls):
        check = FlightLib.selfcheck()
        if not check:
            check = "OK"
        return check

    @classmethod
    def get_position(cls, ros_telemetry):
        x, y, z = ros_telemetry.x, ros_telemetry.y, ros_telemetry.z
        if not math.isnan(x):
            return x, y, z, math.degrees(ros_telemetry.yaw), client.active_client.config.copter_frame_id
        return 'NO_POS'

    def update_telemetry_fast(self):
        self.start_position = self.get_start_position()
        self.last_task = task_manager.get_current_task()
        try:
            self.ros_telemetry = FlightLib.get_telemetry_locked(client.active_client.config.copter_frame_id)
            if self.ros_telemetry.connected:
                self.armed = self.ros_telemetry.armed
                self.mode = self.ros_telemetry.mode
                self.selfcheck = self.get_selfcheck()
                self.current_position = self.get_position(self.ros_telemetry)
            else:
                self.reset_telemetry_values()
        except rospy.ServiceException:
            rospy.logdebug("Some service is unavailable")
            self.selfcheck = ["WAIT_ROS"]
        except AttributeError as e:
            rospy.logdebug(e)
        except rospy.TransportException as e:
            rospy.logdebug(e)
        self.time_delta = time.time()
        self.round_telemetry()

    def update_telemetry_slow(self):
        self.animation_id = animation.get_id()
        self.git_version = self.get_git_version()
        self.config_version = self.get_config_version()
        try:
            self.calibration_status = get_calibration_status()
            self.fcu_status = get_sys_status()
            self.battery = self.get_battery(self.ros_telemetry)
        except rospy.ServiceException:
            rospy.logdebug("Some service is unavailable")
            self.selfcheck = ["WAIT_ROS"]
        except AttributeError as e:
            rospy.logdebug(e)
        except rospy.TransportException as e:
            rospy.logdebug(e)

    def update(self):
        self.update_telemetry_fast()
        self.update_telemetry_slow()

    def round_telemetry(self):
        round_list = ["battery", "start_position", "current_position"]
        for key in round_list:
            if self.__dict__[key] not in [None, 'NO_POS', 'NO_FCU']:
                self.__dict__[key] = [round(v, 2) if type(v) == float else v for v in self.__dict__[key]]

    def reset_telemetry_values(self):
        self.battery = float('nan'), float('nan')
        self.calibration_status = 'NO_FCU'
        self.fcu_status = 'NO_FCU'
        self.mode = 'NO_FCU'
        self.selfcheck = ['NO_FCU']
        self.current_position = 'NO_POS'

    def check_failsafe_and_interruption(self):
        global emergency
        # check current state
        state = [self.mode, self.armed, task_manager.get_last_task_name()]
        mode, armed, last_task = state
        # check external interruption
        external_interruption = (mode != "OFFBOARD" and armed == True and last_task not in [None, 'land'])
        log_msg = ''
        if emergency:
            log_msg += 'emergency and '
        if external_interruption:
            log_msg += 'external interruption and '
            # count interruptions to avoid px4 mode glitches
            if state == self._last_state:
                self._interruption_counter += 1
            else:
                self._interruption_counter = 0
            logger.info("Possible expernal interruption, state_counter = {}".format(self._interruption_counter))
        # delete last ' end ' from log message
        if len(log_msg) > 5:
            log_msg = log_msg[:-5]
        # clear task manager if emergency or external interruption
        if emergency or (external_interruption and self._interruption_counter >= self._max_interruptions):
            if not self._tasks_cleared:
                logger.info("Clear task manager because of {}".format(log_msg))
                logger.info("Mode: {} | armed: {} | last task: {} ".format(mode, armed, last_task))
                task_manager.reset()
                FlightLib.reset_delta()
                self._tasks_cleared = True
                self._interruption_counter = 0
        else:
            self._tasks_cleared = False
        self._last_state = state

    def transmit_message(self):  # todo if connected
        try:
            client.active_client.server_connection.send_message('telemetry', kwargs={'value': self.create_msg_contents()})
        except AttributeError as e:
            logger.debug(e)

    @classmethod
    def log_cpu_and_memory(cls):
        cpu_usage = psutil.cpu_percent(interval=None, percpu=True)
        mem_usage = psutil.virtual_memory().percent
        cpu_temp_info = psutil.sensors_temperatures()['cpu-thermal'][0]
        cpu_temp = cpu_temp_info.current
        # https://github.com/raspberrypi/documentation/blob/JamesH65-patch-vcgencmd-vcdbg-docs/raspbian/applications/vcgencmd.md
        throttled_hex = subprocess.check_output("vcgencmd get_throttled", shell=True).split('=')[1]
        under_voltage = bool(int(bin(int(throttled_hex, 16))[2:][-1]))
        power_state = 'normal' if not under_voltage else 'under voltage!'
        if cpu_temp_info.critical:
            cpu_temp_state = 'critical'
        elif cpu_temp_info.high:
            cpu_temp_state = 'high'
        else:
            cpu_temp_state = 'normal'
        logger.info("CPU usage: {} | Memory: {} % | T: {} ({}) | Power: {}".format(
            cpu_usage, mem_usage, cpu_temp, cpu_temp_state, power_state))

    def _update_loop(self, freq):  # TODO extract?
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():

            self.update_telemetry_fast()
            self.check_failsafe_and_interruption()

            if client.active_client.config.telemetry_transmit and client.active_client.connected:
                self.transmit_message()

            rate.sleep()

    def _slow_update_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update_telemetry_slow()
            if client.active_client.config.telemetry_log_resources:
                self.log_cpu_and_memory()
            rate.sleep()

    def start_loop(self):
        if client.active_client.config.telemetry_frequency > 0:
            telemetry_thread = threading.Thread(target=self._update_loop, name="Telemetry getting thread",
                                                args=(client.active_client.config.telemetry_frequency,))  # TODO MOVE? Daemon?
            slow_telemetry_thread = threading.Thread(target=self._slow_update_loop,
                                                     name="Slow telemetry getting thread")
            slow_telemetry_thread.start()
            telemetry_thread.start()
        else:
            logger.info("Telemetry loop is not created because of zero or negative telemetry frequency")

    def create_msg_contents(self, keys=None):  # keys: set or list
        if keys is None:
            keys = self.params_default_dict.keys()
        # return only existing keys from 'keys'
        return {k: self.__dict__[k] for k in keys if k in self.params_default_dict}


def emergency_callback(data):
    global emergency
    emergency = data.data


if __name__ == "__main__":
    telemetry = Telemetry()
    copter_client = CopterClient()
    task_manager = tasking.TaskManager()
    rospy.Subscriber('/emergency', Bool, emergency_callback)
    copter_client.start(task_manager)
