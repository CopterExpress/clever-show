import os
import sys
import time
import math
import rospy
from clever import srv
import datetime
import logging
import threading
import psutil
import subprocess
import ConfigParser
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
Telemetry = namedtuple("Telemetry", "git_version animation_id battery_v battery_p system_status calibration_status mode selfcheck current_position start_position armed")
telemetry = Telemetry('nan', 'No animation', 'nan', 'nan', 'NO_FCU', 'NO_FCU', 'NO_FCU', 'NO_FCU', 'NO_POS', 'NO_POS', False)
emergency = False

# get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

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
    def load_config(self):
        self.FLOOR_FRAME_EXISTS = False
        super(CopterClient, self).load_config()
        self.TELEM_FREQ = self.config.getfloat('TELEMETRY', 'frequency')
        self.TELEM_TRANSMIT = self.config.getboolean('TELEMETRY', 'transmit')
        self.CLEAR_TASKS_WHEN_EMERGENCY = self.config.getboolean('TELEMETRY', 'clear_tasks_when_emergency')
        self.LOG_CPU_AND_MEMORY = self.config.getboolean('TELEMETRY', 'log_cpu_and_memory')
        self.FRAME_ID = self.config.get('COPTERS', 'frame_id')
        self.FRAME_FLIPPED_HEIGHT = 0.
        self.TAKEOFF_HEIGHT = self.config.getfloat('COPTERS', 'takeoff_height')
        self.TAKEOFF_TIME = self.config.getfloat('COPTERS', 'takeoff_time')
        self.SAFE_TAKEOFF = self.config.getboolean('COPTERS', 'safe_takeoff')
        self.RFP_TIME = self.config.getfloat('COPTERS', 'reach_first_point_time')
        self.LAND_TIME = self.config.getfloat('COPTERS', 'land_time')
        self.LAND_TIMEOUT = self.config.getfloat('COPTERS', 'land_timeout')
        self.X0_COMMON = self.config.getfloat('COPTERS', 'x0_common')
        self.Y0_COMMON = self.config.getfloat('COPTERS', 'y0_common')
        self.Z0_COMMON = self.config.getfloat('COPTERS', 'z0_common')
        self.TAKEOFF_CHECK = self.config.getboolean('ANIMATION', 'takeoff_animation_check')
        self.LAND_CHECK = self.config.getboolean('ANIMATION', 'land_animation_check')
        self.FRAME_DELAY = self.config.getfloat('ANIMATION', 'frame_delay')
        self.X_RATIO = self.config.getfloat('ANIMATION', 'x_ratio')
        self.Y_RATIO = self.config.getfloat('ANIMATION', 'y_ratio')
        self.Z_RATIO = self.config.getfloat('ANIMATION', 'z_ratio')
        self.X0 = self.config.getfloat('PRIVATE', 'x0')
        self.Y0 = self.config.getfloat('PRIVATE', 'y0')
        self.Z0 = self.config.getfloat('PRIVATE', 'z0')
        self.USE_LEDS = self.config.getboolean('PRIVATE', 'use_leds')
        self.LED_PIN = self.config.getint('PRIVATE', 'led_pin')
        try:
            self.FLOOR_DX = self.config.getfloat('FLOOR FRAME', 'x')
            self.FLOOR_DY = self.config.getfloat('FLOOR FRAME', 'y')
            self.FLOOR_DZ = self.config.getfloat('FLOOR FRAME', 'z')
            self.FLOOR_ROLL = self.config.getfloat('FLOOR FRAME', 'roll')
            self.FLOOR_PITCH = self.config.getfloat('FLOOR FRAME', 'pitch')
            self.FLOOR_YAW = self.config.getfloat('FLOOR FRAME', 'yaw')
            self.FLOOR_PARENT = self.config.get('FLOOR FRAME', 'parent')
            self.FLOOR_FRAME_EXISTS = True           
        except ConfigParser.Error:
            rospy.logerror("No floor frame!")
            self.FLOOR_FRAME_EXISTS = False
        self.RESTART_DHCPCD = self.config.getboolean('PRIVATE', 'restart_dhcpcd')

    def on_broadcast_bind(self):
        configure_chrony_ip(self.server_host)
        restart_service("chrony")

    def start(self, task_manager_instance):
        rospy.loginfo("Init ROS node")
        rospy.init_node('clever_show_client')
        if self.USE_LEDS:
            LedLib.init_led(self.LED_PIN)
        task_manager_instance.start()
        if self.FRAME_ID == "floor":
            if self.FLOOR_FRAME_EXISTS: 
                self.start_floor_frame_broadcast()
            else:
                rospy.logerror("Can't make floor frame!")
        start_subscriber()
        telemetry_thread.start()
        super(CopterClient, self).start()

    def start_floor_frame_broadcast(self):
        trans = TransformStamped()
        trans.transform.translation.x = self.FLOOR_DX
        trans.transform.translation.y = self.FLOOR_DY
        trans.transform.translation.z = self.FLOOR_DZ
        trans.transform.rotation = Quaternion(*quaternion_from_euler(math.radians(self.FLOOR_ROLL),
                                                                    math.radians(self.FLOOR_PITCH),
                                                                    math.radians(self.FLOOR_YAW)))
        trans.header.frame_id = self.FLOOR_PARENT
        trans.child_frame_id = self.FRAME_ID
        static_bloadcaster.sendTransform(trans)

def restart_service(name):
    os.system("systemctl restart {}".format(name))

def execute_command(command):
    os.system(command)

def configure_chrony_ip(ip, path="/etc/chrony/chrony.conf", ip_index=1):
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
        return False

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
        content = raw_content[:index_start] + "{}       {} {}.local".format(_ip, hostname, hostname) + raw_content[index_stop:]
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

@messaging.message_callback("id")
def _response_id(*args, **kwargs):
    new_id = kwargs.get("new_id", None)
    if new_id is not None:
        old_id = client.active_client.client_id
        if new_id != old_id:
            cfg = client.ConfigOption("PRIVATE", "id", new_id)
            client.active_client.write_config(True, cfg)
            if new_id != '/hostname':
                if client.active_client.RESTART_DHCPCD:
                    hostname = client.active_client.client_id
                    configure_hostname(hostname)
                    configure_hosts(hostname)
                    configure_bashrc(hostname)
                    configure_motd(hostname)
                    execute_command("reboot")
                    #execute_command("hostname {}".format(hostname))
                    #restart_service("dhcpcd")
                    #restart_service("avahi-daemon")
                    #restart_service("smbd")
                    #restart_service("roscore")
                    #restart_service("clever")
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
    return create_telemetry_message(telemetry)


@messaging.request_callback("anim_id")
def _response_animation_id(*args, **kwargs):
    # Load animation
    result = animation.get_id()
    if result != 'No animation':
        logger.debug ("Saving corrected animation")
        frames = animation.load_animation(os.path.abspath("animation.csv"),
                                            x0=client.active_client.X0 + client.active_client.X0_COMMON,
                                            y0=client.active_client.Y0 + client.active_client.Y0_COMMON,
                                            z0=client.active_client.Z0 + client.active_client.Z0_COMMON,
                                            x_ratio=client.active_client.X_RATIO,
                                            y_ratio=client.active_client.Y_RATIO,
                                            z_ratio=client.active_client.Z_RATIO,
                                            )
        # Correct start and land frames in animation
        corrected_frames, start_action, start_delay = animation.correct_animation(frames,
                                            check_takeoff=client.active_client.TAKEOFF_CHECK,
                                            check_land=client.active_client.LAND_CHECK,
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
    telem = FlightLib.get_telemetry_locked(client.active_client.FRAME_ID)
    return "{:.2f} {:.2f} {:.2f} {:.1f} {}".format(
        telem.x, telem.y, telem.z, math.degrees(telem.yaw), client.active_client.FRAME_ID)

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
                                        x_ratio=client.active_client.X_RATIO,
                                        y_ratio=client.active_client.Y_RATIO,
                                        )
    logger.debug("x_start = {}, y_start = {}".format(x_start, y_start))
    if not math.isnan(x_start):
        telem = FlightLib.get_telemetry_locked(client.active_client.FRAME_ID)
        logger.debug("x_telem = {}, y_telem = {}".format(telem.x, telem.y))
        if not math.isnan(telem.x):
            client.active_client.config.set('PRIVATE', 'x0', telem.x - x_start)
            client.active_client.config.set('PRIVATE', 'y0', telem.y - y_start)
            client.active_client.rewrite_config()
            client.active_client.load_config()
            logger.info ("Set start delta: {:.2f} {:.2f}".format(client.active_client.X0, client.active_client.Y0))
        else:
            logger.debug ("Wrong telemetry")
    else:
        logger.debug("Wrong animation file")

@messaging.message_callback("reset_start")
def _command_reset_start(*args, **kwargs):
    client.active_client.config.set('PRIVATE', 'x0', 0)
    client.active_client.config.set('PRIVATE', 'y0', 0)
    client.active_client.rewrite_config()
    client.active_client.load_config()
    logger.info ("Reset start to {:.2f} {:.2f}".format(client.active_client.X0, client.active_client.Y0))

@messaging.message_callback("set_z_to_ground")
def _command_set_z(*args, **kwargs):
    telem = FlightLib.get_telemetry_locked(client.active_client.FRAME_ID)
    client.active_client.config.set('PRIVATE', 'z0', telem.z)
    client.active_client.rewrite_config()
    client.active_client.load_config()
    logger.info ("Set z offset to {:.2f}".format(client.active_client.Z0))

@messaging.message_callback("reset_z_offset")
def _command_reset_z(*args, **kwargs):
    client.active_client.config.set('PRIVATE', 'z0', 0)
    client.active_client.rewrite_config()
    client.active_client.load_config()
    logger.info ("Reset z offset to {:.2f}".format(client.active_client.Z0))


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
    configure_chrony_ip(client.active_client.server_host)
    restart_service("chrony")


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
    FlightLib.flip(frame_id=client.active_client.FRAME_ID)

@messaging.message_callback("takeoff")
def _command_takeoff(*args, **kwargs):
    logger.info("Takeoff at {}".format(datetime.datetime.now()))
    task_manager.add_task(0, 0, animation.takeoff,
                          task_kwargs={
                              "z": client.active_client.TAKEOFF_HEIGHT,
                              "timeout": client.active_client.TAKEOFF_TIME,
                              "safe_takeoff": client.active_client.SAFE_TAKEOFF,
                              "use_leds": client.active_client.USE_LEDS,
                          }
                          )

@messaging.message_callback("takeoff_z")
def _command_takeoff_z(*args, **kwargs):
    z_str = kwargs.get("z", None)
    if z_str is not None:
        telem = FlightLib.get_telemetry_locked(client.active_client.FRAME_ID)
        logger.info("Takeoff to z = {} at {}".format(z_str, datetime.datetime.now()))
        task_manager.add_task(0, 0, FlightLib.reach_point,
                          task_kwargs={
                              "x": telem.x,
                              "y": telem.y,
                              "z": float(z_str),
                              "frame_id": client.active_client.FRAME_ID,
                              "timeout": client.active_client.TAKEOFF_TIME,
                              "auto_arm": True,
                          }
                          )


@messaging.message_callback("land")
def _command_land(*args, **kwargs):
    task_manager.reset()
    task_manager.add_task(0, 0, animation.land,
                          task_kwargs={
                              "z": client.active_client.TAKEOFF_HEIGHT,
                              "timeout": client.active_client.TAKEOFF_TIME,
                              "frame_id": client.active_client.FRAME_ID,
                              "use_leds": client.active_client.USE_LEDS,
                          }
                          )


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
    
    logger.info("Start time = {}, wait for {} seconds".format(start_time, start_time-time.time()))
    # Load animation
    frames = animation.load_animation(os.path.abspath("animation.csv"),
                                        x0=client.active_client.X0 + client.active_client.X0_COMMON,
                                        y0=client.active_client.Y0 + client.active_client.Y0_COMMON,
                                        z0=client.active_client.Z0 + client.active_client.Z0_COMMON,
                                        x_ratio=client.active_client.X_RATIO,
                                        y_ratio=client.active_client.Y_RATIO,
                                        z_ratio=client.active_client.Z_RATIO,
                                        )
    # Correct start and land frames in animation
    corrected_frames, start_action, start_delay = animation.correct_animation(frames,
                                        check_takeoff=client.active_client.TAKEOFF_CHECK,
                                        check_land=client.active_client.LAND_CHECK,
                                        ) 

    # Choose start action
    if start_action == 'takeoff':
        # Takeoff first
        task_manager.add_task(start_time, 0, animation.takeoff,
                            task_kwargs={
                                "z": client.active_client.TAKEOFF_HEIGHT,
                                "timeout": client.active_client.TAKEOFF_TIME,
                                "safe_takeoff": client.active_client.SAFE_TAKEOFF,
                                # "frame_id": client.active_client.FRAME_ID,
                                "use_leds": client.active_client.USE_LEDS,
                            }
                            )
        # Fly to first point
        rfp_time = start_time + client.active_client.TAKEOFF_TIME
        task_manager.add_task(rfp_time, 0, animation.execute_frame,
                            task_kwargs={
                                "point": animation.convert_frame(corrected_frames[0])[0],
                                "color": animation.convert_frame(corrected_frames[0])[1],
                                "frame_id": client.active_client.FRAME_ID,
                                "use_leds": client.active_client.USE_LEDS,
                                "flight_func": FlightLib.reach_point,
                            }
                            )
        # Calculate first frame start time
        frame_time = rfp_time + client.active_client.RFP_TIME

    elif start_action == 'arm':
        # Calculate start time
        start_time += start_delay
        # Arm
        #task_manager.add_task(start_time, 0, FlightLib.arming_wrapper,
        #                    task_kwargs={
        #                        "state": True
        #                    }
        #                    )
        frame_time = start_time # + 1.0
        point, color, yaw = animation.convert_frame(corrected_frames[0])
        task_manager.add_task(frame_time, 0, animation.execute_frame,
                        task_kwargs={
                            "point": point,
                            "color": color,
                            "frame_id": client.active_client.FRAME_ID,
                            "use_leds": client.active_client.USE_LEDS,
                            "flight_func": FlightLib.navto,
                            "auto_arm": True,
                        }
                        )
        # Calculate first frame start time
        frame_time += client.active_client.FRAME_DELAY # TODO Think about arming time   
    logger.debug(task_manager.task_queue)
    # Play animation file
    for frame in corrected_frames:
        point, color, yaw = animation.convert_frame(frame)
        task_manager.add_task(frame_time, 0, animation.execute_frame,
                        task_kwargs={
                            "point": point,
                            "color": color,
                            "frame_id": client.active_client.FRAME_ID,
                            "use_leds": client.active_client.USE_LEDS,
                            "flight_func": FlightLib.navto,
                        }
                        )
        frame_time += client.active_client.FRAME_DELAY

    # Calculate land_time
    land_time = frame_time + client.active_client.LAND_TIME
    # Land
    task_manager.add_task(land_time, 0, animation.land,
                    task_kwargs={
                        "timeout": client.active_client.LAND_TIMEOUT,
                        "frame_id": client.active_client.FRAME_ID,
                        "use_leds": client.active_client.USE_LEDS,
                    },
                    )

def telemetry_loop():
    global telemetry, emergency
    tasks_cleared = False
    rate = rospy.Rate(client.active_client.TELEM_FREQ)
    while not rospy.is_shutdown():
        telemetry = telemetry._replace(animation_id = animation.get_id())
        telemetry = telemetry._replace(git_version = subprocess.check_output("git log --pretty=format:'%h' -n 1", shell=True))
        x_start, y_start = animation.get_start_xy(os.path.abspath("animation.csv"),
                                                    x_ratio=client.active_client.X_RATIO,
                                                    y_ratio=client.active_client.Y_RATIO,
                                                    )
        x_delta = client.active_client.X0 + client.active_client.X0_COMMON
        y_delta = client.active_client.Y0 + client.active_client.Y0_COMMON
        z_delta = client.active_client.Z0 + client.active_client.Z0_COMMON
        if not math.isnan(x_start):
            telemetry = telemetry._replace(start_position = '{:.2f} {:.2f} {:.2f}'.format(x_start+x_delta, y_start+y_delta, z_delta))
        else:
            telemetry = telemetry._replace(start_position = 'NO_POS')
        services_unavailable = FlightLib.check_ros_services_unavailable()
        if not services_unavailable:
            try:
                ros_telemetry = FlightLib.get_telemetry_locked(client.active_client.FRAME_ID)
                if ros_telemetry.connected:
                    telemetry = telemetry._replace(armed = ros_telemetry.armed)
                    telemetry = telemetry._replace(battery_v = '{:.2f}'.format(ros_telemetry.voltage))
                    batt_empty_param = get_param('BAT_V_EMPTY')
                    batt_charged_param = get_param('BAT_V_CHARGED')
                    batt_cells_param = get_param('BAT_N_CELLS')
                    if batt_empty_param.success and batt_charged_param.success and batt_cells_param.success:
                        batt_empty = batt_empty_param.value.real
                        batt_charged = batt_charged_param.value.real
                        batt_cells = batt_cells_param.value.integer
                        try:
                            telemetry = telemetry._replace(battery_p = '{}'.format(int(min((ros_telemetry.voltage/batt_cells - batt_empty)/(batt_charged - batt_empty)*100., 100))))
                        except ValueError:
                            telemetry = telemetry._replace(battery_p = 'nan')
                    else:
                        telemetry = telemetry._replace(battery_p = 'nan')
                    telemetry = telemetry._replace(calibration_status = get_calibration_status())
                    telemetry = telemetry._replace(system_status = get_sys_status())
                    telemetry = telemetry._replace(mode = ros_telemetry.mode)
                    check = FlightLib.selfcheck()
                    if not check:
                        check = "OK"
                    telemetry = telemetry._replace(selfcheck = str(check))    
                    if not math.isnan(ros_telemetry.x):
                        telemetry = telemetry._replace(current_position = '{:.2f} {:.2f} {:.2f} {:.1f} {}'.format(ros_telemetry.x, ros_telemetry.y, ros_telemetry.z, 
                                                                                        math.degrees(ros_telemetry.yaw), client.active_client.FRAME_ID))
                    else: 
                        telemetry = telemetry._replace(current_position = 'NO_POS in {}'.format(client.active_client.FRAME_ID))
                else:
                    telemetry = telemetry._replace(battery_v = 'nan')
                    telemetry = telemetry._replace(battery_p = 'nan')
                    telemetry = telemetry._replace(calibration_status = 'NO_FCU')
                    telemetry = telemetry._replace(system_status = 'NO_FCU')
                    telemetry = telemetry._replace(mode = 'NO_FCU')
                    telemetry = telemetry._replace(selfcheck = 'NO_FCU')
                    telemetry = telemetry._replace(current_position = 'NO_POS')
            except rospy.ServiceException:
                logger.debug("Some service is unavailable")
            except AttributeError as e:
                logger.debug(e)
            except rospy.TransportException as e:
                logger.debug(e)
        else:
            telemetry = telemetry._replace(selfcheck = 'WAIT_ROS')
        if client.active_client.TELEM_TRANSMIT:
            try:
                client.active_client.server_connection.send_message('telem', args={'message':create_telemetry_message(telemetry)})
            except AttributeError as e:
                logger.debug(e)
        if client.active_client.CLEAR_TASKS_WHEN_EMERGENCY:
            mode = telemetry.mode
            armed = telemetry.armed
            last_task = task_manager.get_last_task_name()
            external_interruption = (mode != "OFFBOARD" and armed == True and last_task not in [None, 'land'])
            log_msg = ''
            if emergency and external_interruption:
                log_msg = "emergency and external interruption"
            elif emergency:
                log_msg = "emergency"
            elif external_interruption:
                log_msg = "external interruption"
            if emergency or external_interruption:
                if not tasks_cleared:
                    logger.info("Clear task manager because of {}".format(log_msg))
                    logger.info("Mode: {} | armed: {} | last task: {}".format(mode, armed, last_task))
                    task_manager.reset()
                    tasks_cleared = True
            else:
                tasks_cleared = False
        if client.active_client.LOG_CPU_AND_MEMORY:
            cpu_usage = psutil.cpu_percent(interval=None, percpu=True)
            mem_usage = psutil.virtual_memory().percent
            cpu_temp_info = psutil.sensors_temperatures()['cpu-thermal'][0]
            cpu_temp = cpu_temp_info.current
            if cpu_temp_info.critical:
                cpu_temp_state = 'critical'
            elif cpu_temp_info.high:
                cpu_temp_state = 'high'
            else:
                cpu_temp_state = 'normal'
            logger.info("CPU usage: {} | Memory: {} % | T: {} ({})".format(cpu_usage, mem_usage, cpu_temp, cpu_temp_state))

        rate.sleep()

def create_telemetry_message(telemetry):
    msg = client.active_client.client_id + '`'
    for key in telemetry.__dict__:
        if key != 'armed':
            msg += telemetry.__dict__[key] + '`'
    msg += repr(time.time())
    return msg 

def emergency_callback(data):
    global emergency
    emergency = data.data

telemetry_thread = threading.Thread(target=telemetry_loop, name="Telemetry getting thread")

if __name__ == "__main__":
    
    copter_client = CopterClient()
    task_manager = tasking.TaskManager()
    rospy.Subscriber('/emergency', Bool, emergency_callback)
    copter_client.start(task_manager)
