import os
import sys
import time
import math
import numpy
import psutil
import logging
import datetime
import threading
import subprocess
from collections import namedtuple
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# Import rospy
try:
    import rospy
except ImportError:
    print("Can't import rospy! Please check your ROS installation.")
    exit()

import rospkg

# Import clever or clover package
try:
    from clever import srv
except ImportError:
    try:
        from clover import srv
    except ImportError:
        print("Can't import clever or clover! Please check installation of clover ROS package.")
        exit()

# Import flight control
try:
    import modules.flight as flight
except ImportError:
    print("Can't import flight control module!")

# Import led control
try:
    import modules.led as led
except ImportError:
    print("Can't import led control module!")

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = (os.path.dirname(os.path.realpath(__file__)))
lib_dir = os.path.realpath(os.path.join(current_dir, '../lib'))
sys.path.insert(0, lib_dir)

import messaging
import modules.client_core as client_core
import modules.animation as animation
import modules.mavros_wrapper as mavros
import modules.tasking as tasking

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import tf2_ros
from geographiclib.geodesic import Geodesic

Earth = Geodesic.WGS84

def dist(x, y):
    return math.sqrt(x**2+y**2)

def azi(x, y):
    return 90 - math.atan2(y,x)*180/math.pi

def get_xy(dist, azi):
    return dist*math.sin(math.radians(azi)), dist*math.cos(math.radians(azi))

def valid(pos):
    for coord in pos:
        if math.isnan(coord):
            return False
    return True

def contains_nan(array):
    for num in array:
        if math.isnan(num):
            return True
    return False

static_broadcaster = tf2_ros.StaticTransformBroadcaster()

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

animation_logger = logging.getLogger('modules.animation')
animation_logger.setLevel(logging.INFO)
animation_logger.addHandler(handler)

client_logger = logging.getLogger('modules.client_core')
client_logger.setLevel(logging.DEBUG)
client_logger.addHandler(handler)

messaging_logger = logging.getLogger('messaging')
messaging_logger.setLevel(logging.INFO)
messaging_logger.addHandler(handler)

tasking_logger = logging.getLogger('modules.tasking')
tasking_logger.setLevel(logging.INFO)
tasking_logger.addHandler(handler)

flight_logger = logging.getLogger('modules.flight')
flight_logger.setLevel(logging.INFO)
flight_logger.addHandler(handler)

mavros_mavlink_logger = logging.getLogger('modules.mavros_wrapper')
mavros_mavlink_logger.setLevel(logging.INFO)
mavros_mavlink_logger.addHandler(handler)


class CopterClient(client_core.Client):
    def __init__(self, config_path="config/client.ini"):
        super(CopterClient, self).__init__(config_path)
        self.load_config()
        if self.config.clover_dir == 'auto':
            self.check_clover_dir()
        self.telemetry = None
        self.animation = animation.Animation("animation.csv", self.config)
        self.gps_thread_run = False
        self.gps_thread_is_running = False

    def load_config(self):
        super(CopterClient, self).load_config()

    def check_clover_dir(self):
        rospack = rospkg.RosPack()
        try:
            path = rospack.get_path('clever')
        except rospkg.common.ResourceNotFound:
            try:
                path = rospack.get_path('clover')
            except rospkg.common.ResourceNotFound:
                path = 'error'
        self.config.set('', 'clover_dir', path, write=True)
        if path.count("/pi/"):
            self.server_connection.whoami = "pi"

    def on_broadcast_bind(self):
        repair_chrony(self.config.server_host)

    def start(self, task_manager_instance):
        rospy.loginfo("Init ROS node")
        rospy.init_node('clever_show_client', anonymous=True)
        task_manager_instance.start()
        mavros.start_subscriber()
        self.telemetry = Telemetry()
        self.telemetry.start_loop()
        if self.config.flight_frame_id == "floor":
            self.start_floor_frame_broadcast()
        elif self.config.flight_frame_id == "gps":
            self.start_gps_frame_broadcast()
        client_thread = threading.Thread(target=super(CopterClient, self).start, name="Client thread")
        client_thread.daemon = True
        client_thread.start()

    def on_config_update(self):
        self.gps_thread_run = False
        self.load_config()
        self.animation.on_config_update(self.config)
        if self.config.flight_frame_id == "floor":
            self.start_floor_frame_broadcast()
        elif self.config.flight_frame_id == "gps":
            self.start_gps_frame_broadcast()

    def start_floor_frame_broadcast(self):
        if self.config.floor_frame_parent == "gps":
            self.start_gps_frame_broadcast()
        trans = TransformStamped()
        trans.transform.translation.x = self.config.floor_frame_translation[0]
        trans.transform.translation.y = self.config.floor_frame_translation[1]
        trans.transform.translation.z = self.config.floor_frame_translation[2]
        trans.transform.rotation = Quaternion(*quaternion_from_euler(math.radians(self.config.floor_frame_rotation[0]),
                                                                     math.radians(self.config.floor_frame_rotation[1]),
                                                                     math.radians(self.config.floor_frame_rotation[2])))
        trans.header.frame_id = self.config.floor_frame_parent
        trans.child_frame_id = self.config.flight_frame_id
        static_broadcaster.sendTransform(trans)

    def start_gps_frame_broadcast(self):
        gps_frame_thread = threading.Thread(target=self.gps_frame_broadcast_loop, name="GPS frame broadcast thread")
        gps_frame_thread.daemon = True
        gps_frame_thread.start()

    def gps_frame_broadcast_loop(self):
        while self.gps_thread_is_running:
            rospy.sleep(1)
            logger.info("Wait until previous gps thread stop")
        self.gps_thread_run = True
        self.gps_thread_is_running = True
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and self.gps_thread_run:
            telem = flight.get_telemetry_locked(frame_id = "map")
            amsl_height = mavros.get_amsl_altitude()
            compass_hdg = mavros.get_compass_hdg()
            if telem is not None:
                if contains_nan([telem.lat, telem.lon, telem.x, telem.y, telem.yaw, amsl_height, compass_hdg]):
                    logger.info("Can't get position data from GPS")
                else:
                    lat = float(self.config.gps_frame_lat)
                    lon = float(self.config.gps_frame_lon)
                    geo_delta = Earth.Inverse(telem.lat, telem.lon, lat, lon)
                    #logger.info("dist: {} | azi: {}".format(geo_delta['s12'], geo_delta['azi1']))
                    dx, dy = get_xy(geo_delta['s12'], geo_delta['azi1'])
                    gps_dx = telem.x + dx
                    gps_dy = telem.y + dy
                    gps_dz = 0
                    if self.config.gps_frame_amsl != "current":
                        try:
                            gps_dz = telem.z - amsl_height + float(self.config.gps_frame_amsl)
                        except ValueError:
                            logger.error("Wrong amsl height in GPS FRAME config!")
                            continue
                    #logger.info("GPS frame dx: {} | dy: {}".format(gps_dx, gps_dy))
                    trans = TransformStamped()
                    trans.transform.translation.x = gps_dx
                    trans.transform.translation.y = gps_dy
                    trans.transform.translation.z = gps_dz
                    yaw = math.radians(compass_hdg) + telem.yaw - math.radians(self.config.gps_frame_yaw)
                    trans.transform.rotation = Quaternion(*quaternion_from_euler(0,0,yaw))
                    trans.header.frame_id = "map"
                    trans.child_frame_id = "gps"
                    static_broadcaster.sendTransform(trans)
            rate.sleep()
        self.gps_thread_is_running = False


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
        old_id = copter.client_id
        if new_id != old_id:
            copter.config.set('', 'id', new_id, write=True)
            copter.client_id = new_id
            if new_id != '/hostname':
                if copter.config.system_restart_after_rename:
                    hostname = copter.client_id
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
    if mavros.check_state_topic(wait_new_status=True):
        check = flight.selfcheck()
        return check if check else "OK"
    else:
        mavros.stop_subscriber()
        return "NOT_CONNECTED_TO_FCU"


@messaging.request_callback("telemetry")
def _response_telemetry(*args, **kwargs):
    copter.telemetry.update()
    return copter.telemetry.create_msg_contents()


@messaging.request_callback("anim_id")
def _response_animation_id(*args, **kwargs):
    # Load animation
    return copter.animation.id


@messaging.request_callback("batt_voltage")
def _response_batt(*args, **kwargs):
    if mavros.check_state_topic(wait_new_status=True):
        return flight.get_telemetry_locked('body').voltage
    else:
        mavros.stop_subscriber()
        return float('nan')


@messaging.request_callback("cell_voltage")
def _response_cell(*args, **kwargs):
    if mavros.check_state_topic(wait_new_status=True):
        return flight.get_telemetry_locked('body').cell_voltage
    else:
        mavros.stop_subscriber()
        return float('nan')


@messaging.request_callback("sys_status")
def _response_sys_status(*args, **kwargs):
    return mavros.get_sys_status()


@messaging.request_callback("cal_status")
def _response_cal_status(*args, **kwargs):
    if mavros.check_state_topic(wait_new_status=True):
        return mavros.get_calibration_status()
    else:
        mavros.stop_subscriber()
        return "NOT_CONNECTED_TO_FCU"


@messaging.request_callback("position")
def _response_position(*args, **kwargs):
    telem = copter.telemetry.ros_telemetry
    return "{:.2f} {:.2f} {:.2f} {:.1f} {}".format(
        telem.x, telem.y, telem.z, math.degrees(telem.yaw), copter.config.flight_frame_id)


@messaging.request_callback("calibrate_gyro")
def _calibrate_gyro(*args, **kwargs):
    mavros.calibrate('gyro')
    return mavros.get_calibration_status()


@messaging.request_callback("calibrate_level")
def _calibrate_level(*args, **kwargs):
    mavros.calibrate('level')
    return mavros.get_calibration_status()


@messaging.request_callback("load_params")
def _load_params(*args, **kwargs):
    result = mavros.load_param_file('temp.params')
    logger.info("Load parameters to FCU success: {}".format(result))
    return result


@messaging.message_callback("test")
def _command_test(*args, **kwargs):
    logger.info("logging info test")
    rospy.logdebug("ros logdebug test")
    print("stdout test")


@messaging.message_callback("update_animation")
def _command_update_animation(*args, **kwargs):
    pass # should be updated via watchdog event
    #copter.animation.update_frames(copter.config, "animation.csv")


@messaging.message_callback("move_start")
def _command_move_start_to_current_position(*args, **kwargs):
    private_offset = copter.config.animation_private_offset
    offset = numpy.array(private_offset) + numpy.array(copter.config.animation_common_offset)
    try:
        xs, ys, zs = copter.animation.get_start_frame(copter.telemetry.start_action).get_pos()
    except ValueError:
        logger.error("Can't get start point. Check animation file!")
    else:
        logger.debug("start x = {}, y = {}".format(xs, ys))
        telem = copter.telemetry.ros_telemetry
        logger.debug("telemetry x = {}, y = {}".format(telem.x, telem.y))
        if valid([telem.x, telem.y, telem.z]):
            copter.config.set('ANIMATION', 'private_offset',
                [private_offset[0] + telem.x - xs, private_offset[1] + telem.y - ys, private_offset[2]], write=True)
            logger.info("Set start delta: {:.2f} {:.2f}".format(copter.config.animation_private_offset[0],
                                                                copter.config.animation_private_offset[1]))
        else:
            logger.error("Wrong telemetry")


@messaging.message_callback("reset_start")
def _command_reset_start(*args, **kwargs):
    copter.config.set('ANIMATION', 'private_offset', [0, 0, copter.config.animation_private_offset[2]], write=True)
    logger.info("Reset start to {:.2f} {:.2f}".format(copter.config.animation_private_offset[0], copter.config.animation_private_offset[1]))


@messaging.message_callback("set_z_to_ground")
def _command_set_z(*args, **kwargs):
    telem = copter.telemetry.ros_telemetry
    if valid([telem.x, telem.y, telem.z]):
        copter.config.set('ANIMATION', 'private_offset',
            [copter.config.animation_private_offset[0], copter.config.animation_private_offset[1], telem.z], write=True)
        logger.info("Set z offset to {:.2f}".format(copter.config.animation_private_offset[2]))
    else:
        logger.error("Wrong telemetry")


@messaging.message_callback("reset_z_offset")
def _command_reset_z(*args, **kwargs):
    copter.config.set('ANIMATION', 'private_offset',
        [copter.config.animation_private_offset[0], copter.config.animation_private_offset[1], 0], write=True)
    logger.info("Reset z offset to {:.2f}".format(copter.config.animation_private_offset[2]))


@messaging.message_callback("update_repo")
def _command_update_repo(*args, **kwargs):
    os.system("git fetch")
    os.system("git pull --rebase")
    os.system("chown -R pi:pi /home/pi/clever-show")


@messaging.message_callback("reboot_all")
def _command_reboot_all(*args, **kwargs):
    mavros.reboot_fcu()
    execute_command("reboot")


@messaging.message_callback("reboot_fcu")
def _command_reboot(*args, **kwargs):
    mavros.reboot_fcu()


@messaging.message_callback("service_restart")
def _command_service_restart(*args, **kwargs):
    service = kwargs["name"]
    if service=="clover":
        restart_service("clever")
        restart_service("clover")
    if service=="clever-show":
        restart_service("clever-show@{}".format(copter.client_id))
    restart_service(service)


@messaging.message_callback("repair_chrony")
def _command_chrony_repair(*args, **kwargs):
    repair_chrony(copter.config.server_host)


@messaging.message_callback("led_test")
def _command_led_test(*args, **kwargs):
    led.set_effect(effect='flash', r=255, g=255, b=255)


@messaging.message_callback("led_fill")
def _command_led_fill(*args, **kwargs):
    red = kwargs.get("red", 0)
    green = kwargs.get("green", 0)
    blue = kwargs.get("blue", 0)
    led.set_effect(r=red, g=green, b=blue)


@messaging.message_callback("flip")
def _copter_flip(*args, **kwargs):
    flight.flip(frame_id=copter.config.flight_frame_id)


@messaging.message_callback("takeoff")
def _command_takeoff(*args, **kwargs):
    logger.info("Takeoff at {}".format(datetime.datetime.now()))
    task_manager.add_task(0, 0, animation.takeoff,
                          task_kwargs={
                              "z": copter.config.flight_takeoff_height,
                              "timeout": copter.config.flight_takeoff_time,
                              "safe_takeoff": False,
                              "use_leds": copter.config.led_use & copter.config.led_takeoff_indication,
                          })


@messaging.message_callback("takeoff_z")
def _command_takeoff_z(*args, **kwargs):
    try:
        z = float(kwargs.get("z", None))
    except TypeError:
        logger.error("takeoff_z: No z argument!")
    except ValueError:
        logger.error("takeoff_z: Wrong z argument!")
    else:
        telem = flight.get_telemetry_locked(copter.config.flight_frame_id)
        if valid([telem.x, telem.y, telem.z]):
            logger.info("Takeoff to z = {} at {}".format(z, datetime.datetime.now()))
            task_manager.add_task(0, 0, flight.reach_point,
                                task_kwargs={
                                    "x": telem.x,
                                    "y": telem.y,
                                    "z": z,
                                    "frame_id": copter.config.flight_frame_id,
                                    "timeout": copter.config.flight_takeoff_time,
                                    "auto_arm": True,
                                })
        else:
            logger.error("Wrong telemetry!")


@messaging.message_callback("land")
def _command_land(*args, **kwargs):
    task_manager.reset()
    task_manager.add_task(0, 0, animation.land,
                          task_kwargs={
                              "z": copter.config.flight_takeoff_height,
                              "timeout": copter.config.flight_land_timeout,
                              "frame_id": copter.config.flight_frame_id,
                              "use_leds": copter.config.led_use & copter.config.led_land_indication,
                          })


@messaging.message_callback("emergency_land")
def _emergency_land(*args, **kwargs):
    try:
        result = flight.emergency_land()
        logger.info(result.message)
    except rospy.ServiceException:
        logger.error("Can't execute emergency land: service is unavailable!")


@messaging.message_callback("disarm")
def _command_disarm(*args, **kwargs):
    task_manager.reset()
    task_manager.add_task(-5, 0, flight.arming_wrapper,
                          task_kwargs={
                              "state": False
                          })


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

    # Validate start_time
    try:
        start_time = float(kwargs["time"])
    except ValueError:
        logger.error("start: Wrong time argument!")
        return
    except KeyError:
        logger.error("start: No time argument!")
        return

    # Check animation state
    if copter.animation.state is not "OK":
        logger.error("start: Bad animation state")
        return

    # Get output frames
    frames = copter.animation.get_output_frames(copter.telemetry.start_action)
    if not frames:
        logger.error("start: No frames in animation!")
        return

    # Get current telemetry
    # telem = copter.telemetry.ros_telemetry
    # if not valid([telem.x, telem.y, telem.z]):
    #     logger.error("start: Position is not valid!")
    #     return

    # Play animation!
    frame_time = start_time
    for frame in frames:
        task_manager.add_task(frame_time, 0, animation.execute_frame,
                              task_kwargs={
                                  "frame": frame,
                                  "config": copter.config,
                              })
        frame_time += frame.delay
    task_manager.add_task(frame_time, 0, animation.turn_off_led)

# noinspection PyAttributeOutsideInit
class Telemetry:
    params_default_dict = {
        "git_version": None,
        "animation_info": None,
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
        self.start_action = None

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
        return "{} V{}".format(copter.config.config_name, copter.config.config_version)

    def get_start_position(self):
        try:
            x, y, z = copter.animation.get_start_frame('fly').get_pos()
        except (ValueError, AttributeError):
            return [float('nan'),float('nan'),float('nan'),float('nan'),"error: can't get start pos in animation",float('nan')]
        else:
            start_delay = copter.animation.start_time
            yaw = copter.animation.get_start_frame('fly').yaw
            if not self.ros_telemetry:
                self.start_action = 'error: no telemetry data'
            else:
                self.start_action = copter.animation.get_start_action(self.ros_telemetry.z, self.fcu_status)
            return [x,y,z,yaw,self.start_action,start_delay]

    @classmethod
    def get_battery(cls, ros_telemetry):
        if ros_telemetry is None:
            return float('nan'), float('nan')

        battery_v = ros_telemetry.voltage

        batt_empty_param = mavros.get_param('BAT_V_EMPTY')
        batt_charged_param = mavros.get_param('BAT_V_CHARGED')
        batt_cells_param = mavros.get_param('BAT_N_CELLS')

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
        check = flight.selfcheck()
        if not check:
            check = "OK"
        return check

    @classmethod
    def get_position(cls, ros_telemetry):
        try:
            x, y, z = ros_telemetry.x, ros_telemetry.y, ros_telemetry.z
        except AttributeError:
            return 'NO_POS'
        if not math.isnan(x):
            return x, y, z, math.degrees(ros_telemetry.yaw), copter.config.flight_frame_id
        return 'NO_POS'

    def get_ros_telemetry(self):
        return self.ros_telemetry

    def update_telemetry_fast(self):
        self.last_task = task_manager.get_current_task()
        try:
            self.ros_telemetry = flight.get_telemetry_locked(copter.config.flight_frame_id)
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
        self.animation_info = [copter.animation.id, copter.animation.state]
        self.git_version = self.get_git_version()
        self.config_version = self.get_config_version()
        self.start_position = self.get_start_position()
        try:
            self.calibration_status = mavros.get_calibration_status()
            self.fcu_status = mavros.get_sys_status()
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
        external_interruption = (mode != "OFFBOARD" and armed == True and last_task not in [None, 'land', 'stand'])
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
                flight.reset_delta()
                self._tasks_cleared = True
                self._interruption_counter = 0
        else:
            self._tasks_cleared = False
        self._last_state = state

    def transmit_message(self):  # todo if connected
        try:
            copter.server_connection.send_message('telemetry', kwargs={'value': self.create_msg_contents()})
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

            if copter.config.telemetry_transmit and copter.connected:
                self.transmit_message()

            rate.sleep()

    def _slow_update_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update_telemetry_slow()
            if copter.config.telemetry_log_resources:
                self.log_cpu_and_memory()
            rate.sleep()

    def start_loop(self):
        if copter.config.telemetry_frequency > 0:
            telemetry_thread = threading.Thread(target=self._update_loop, name="Telemetry getting thread",
                                                args=(copter.config.telemetry_frequency,))
            telemetry_thread.daemon = True
            telemetry_thread.start()
            slow_telemetry_thread = threading.Thread(target=self._slow_update_loop,
                                                     name="Slow telemetry getting thread")
            slow_telemetry_thread.daemon = True
            slow_telemetry_thread.start()
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


class AnimationEventHandler(FileSystemEventHandler):
    def on_any_event(self, event):
        logger.info('{} is {}'.format(event.src_path, event.event_type))
        # logger.info(os.path.splitext(event.src_path))
        if event.src_path.split('/')[-1] == 'client.ini':
            if os.path.exists("animation.csv"):
                copter.on_config_update()
                logger.info("Config updated!")
        elif (os.path.splitext(event.src_path)[-1] == '.csv' and event.event_type != "deleted"):
            if os.path.exists("animation.csv"):
                copter.animation.on_animation_update("animation.csv")
                logger.info("Animation updated!")


if __name__ == "__main__":
    copter = CopterClient()
    task_manager = tasking.TaskManager()
    rospy.Subscriber('/emergency', Bool, emergency_callback)
    event_handler = AnimationEventHandler()
    observer = Observer()
    observer.schedule(event_handler, ".", recursive=True)
    observer.daemon = True
    observer.start()
    copter.start(task_manager)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
