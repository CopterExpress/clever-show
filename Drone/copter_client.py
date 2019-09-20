import os
import time
import math
import rospy
import logging

from FlightLib import FlightLib
from FlightLib import LedLib

import client

import messaging_lib as messaging
import tasking_lib as tasking
import animation_lib as animation

from mavros_mavlink import *

from geometry_msgs.msg import Point, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import tf2_ros

static_bloadcaster = tf2_ros.StaticTransformBroadcaster()

# logging.basicConfig(  # TODO all prints as logs
#    level=logging.DEBUG, # INFO
#    format="%(asctime)s [%(name)-7.7s] [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s",
#    handlers=[
#        logging.StreamHandler(),
#    ])

logger = logging.getLogger(__name__)


# import ros_logging

class CopterClient(client.Client):
    def load_config(self):
        super(CopterClient, self).load_config()
        self.FRAME_ID = self.config.get('COPTERS', 'frame_id')
        self.FRAME_FLIPPED_HEIGHT = 0.
        self.TAKEOFF_HEIGHT = self.config.getfloat('COPTERS', 'takeoff_height')
        self.TAKEOFF_TIME = self.config.getfloat('COPTERS', 'takeoff_time')
        self.SAFE_TAKEOFF = self.config.getboolean('COPTERS', 'safe_takeoff')
        self.RFP_TIME = self.config.getfloat('COPTERS', 'reach_first_point_time')
        self.LAND_TIME = self.config.getfloat('COPTERS', 'land_time')
        self.X0_COMMON = self.config.getfloat('COPTERS', 'x0_common')
        self.Y0_COMMON = self.config.getfloat('COPTERS', 'y0_common')
        self.TAKEOFF_CHECK = self.config.getboolean('ANIMATION', 'takeoff_animation_check')
        self.LAND_CHECK = self.config.getboolean('ANIMATION', 'land_animation_check')
        self.FRAME_DELAY = self.config.getfloat('ANIMATION', 'frame_delay')
        self.X0 = self.config.getfloat('PRIVATE', 'x0')
        self.Y0 = self.config.getfloat('PRIVATE', 'y0')
        self.USE_LEDS = self.config.getboolean('PRIVATE', 'use_leds')
        self.LED_PIN = self.config.getint('PRIVATE', 'led_pin')

    def on_broadcast_bind(self):
        configure_chrony_ip(self.server_host)
        restart_service("chrony")

    def start(self, task_manager_instance):
        client.logger.info("Init ROS node")
        rospy.init_node('Swarm_client')
        if self.USE_LEDS:
            LedLib.init_led(self.LED_PIN)
        task_manager_instance.start()
        if self.FRAME_ID == "aruco_map_flipped":
            try:
                self.FRAME_FLIPPED_HEIGHT = self.config.getfloat('COPTERS', 'frame_flipped_height')
                self.FRAME_FLIPPED_WIDTH = self.config.getfloat('COPTERS', 'frame_flipped_width')
            except Exception as e:
                pass
            else:
                trans = TransformStamped()
                trans.transform.translation.x = 0.
                trans.transform.translation.y = self.FRAME_FLIPPED_WIDTH
                trans.transform.translation.z = self.FRAME_FLIPPED_HEIGHT
                trans.transform.rotation = Quaternion(*quaternion_from_euler(math.pi,0,0))
                trans.header.frame_id = "aruco_map"
                trans.child_frame_id = "aruco_map_flipped"
                static_bloadcaster.sendTransform(trans)
        start_subscriber()
        # print(check_state_topic())
        super(CopterClient, self).start()


def restart_service(name):
    os.system("systemctl restart {}".format(name))

def configure_chrony_ip(ip, path="/etc/chrony/chrony.conf", ip_index=1):
    try:
        with open(path, 'r') as f:
            raw_content = f.read()
    except IOError as e:
        print("Reading error {}".format(e))
        return False

    content = raw_content.split(" ")

    try:
        current_ip = content[ip_index]
    except IndexError:
        print("Something wrong with config")
        return False

    if "." not in current_ip:
        print("That's not ip!")
        return False

    if current_ip != ip:
        content[ip_index] = ip

        try:
            with open(path, 'w') as f:
                f.write(" ".join(content))
        except IOError:
            print("Error writing")
            return False

    return True


@messaging.request_callback("selfcheck")
def _response_selfcheck():
    if check_state_topic(wait_new_status=True):
        check = FlightLib.selfcheck()
        return check if check else "OK"
    else:
        stop_subscriber()
        return "NOT_CONNECTED_TO_FCU"


@messaging.request_callback("anim_id")
def _response_animation_id():
    # Load animation
    result = animation.get_id()
    if result != 'No animation':
        print ("Saving corrected animation")
        frames = animation.load_animation(os.path.abspath("animation.csv"),
                                            x0=client.active_client.X0 + client.active_client.X0_COMMON,
                                            y0=client.active_client.Y0 + client.active_client.Y0_COMMON,
                                            )
        # Correct start and land frames in animation
        corrected_frames, start_action, start_delay = animation.correct_animation(frames,
                                            check_takeoff=client.active_client.TAKEOFF_CHECK,
                                            check_land=client.active_client.LAND_CHECK,
                                            )
        print("Start action: {}".format(start_action))
        # Save corrected animation
        animation.save_corrected_animation(corrected_frames)
    return result

@messaging.request_callback("batt_voltage")
def _response_batt():
    if check_state_topic(wait_new_status=True):
        return FlightLib.get_telemetry('body').voltage
    else:
        stop_subscriber()
        return float('nan')


@messaging.request_callback("cell_voltage")
def _response_cell():
    if check_state_topic(wait_new_status=True):
        return FlightLib.get_telemetry('body').cell_voltage
    else:
        stop_subscriber()
        return float('nan')

@messaging.request_callback("sys_status")
def _response_sys_status():
    return get_sys_status()

@messaging.request_callback("cal_status")
def _response_cal_status():
    return get_calibration_status()

@messaging.request_callback("position")
def _response_position():
    telem = FlightLib.get_telemetry(client.active_client.FRAME_ID)
    return "{:.2f} {:.2f} {:.2f} {:.1f} {}".format(
        telem.x, telem.y, telem.z, math.degrees(telem.yaw), client.active_client.FRAME_ID)

@messaging.request_callback("calibrate_gyro")
def _calibrate_gyro():
    calibrate('gyro')
    return get_calibration_status()

@messaging.request_callback("calibrate_level")
def _calibrate_level():
    calibrate('level')
    return get_calibration_status()


@messaging.message_callback("test")
def _command_test(**kwargs):
    logger.info("logging info test")
    print("stdout test")

@messaging.message_callback("move_start")
def _command_move_start_to_current_position(**kwargs):
    # Load animation
    frames = animation.load_animation(os.path.abspath("animation.csv"),
                                        x0=client.active_client.X0_COMMON,
                                        y0=client.active_client.Y0_COMMON,
                                        )
    # Correct start and land frames in animation
    corrected_frames, start_action, start_delay = animation.correct_animation(frames,
                                        check_takeoff=client.active_client.TAKEOFF_CHECK,
                                        check_land=client.active_client.LAND_CHECK,
                                        )
    x_start = corrected_frames[0]['x']
    y_start = corrected_frames[0]['y']
    telem = FlightLib.get_telemetry(client.active_client.FRAME_ID)
    client.active_client.config.set('PRIVATE', 'x0', telem.x - x_start)
    client.active_client.config.set('PRIVATE', 'y0', telem.y - y_start)
    client.active_client.rewrite_config()
    client.active_client.load_config()
    print ("Start delta: {:.2f} {:.2f}".format(client.active_client.X0, client.active_client.Y0))

@messaging.message_callback("reset_start")
def _command_reset_start(**kwargs):
    client.active_client.config.set('PRIVATE', 'x0', 0)
    client.active_client.config.set('PRIVATE', 'y0', 0)
    client.active_client.rewrite_config()
    client.active_client.load_config()
    print ("Reset start to {:.2f} {:.2f}".format(client.active_client.X0, client.active_client.Y0))


@messaging.message_callback("update_repo")
def _command_update_repo(**kwargs):
    os.system("git reset --hard origin/master")
    os.system("git fetch")
    os.system("git pull")
    os.system("chown -R pi:pi ~/CleverSwarm")

@messaging.message_callback("reboot_fcu")
def _command_reboot():
    reboot_fcu()


@messaging.message_callback("service_restart")
def _command_service_restart(**kwargs):
    restart_service(kwargs["name"])

@messaging.message_callback("repair_chrony")
def _command_chrony_repair():
    configure_chrony_ip(client.active_client.server_host)
    restart_service("chrony")


@messaging.message_callback("led_test")
def _command_led_test(**kwargs):
    LedLib.chase(255, 255, 255)
    time.sleep(2)
    LedLib.off()


@messaging.message_callback("led_fill")
def _command_led_fill(**kwargs):
    r = kwargs.get("red", 0)
    g = kwargs.get("green", 0)
    b = kwargs.get("blue", 0)

    LedLib.fill(r, g, b)


@messaging.message_callback("flip")
def _copter_flip():
    FlightLib.flip(frame_id=client.active_client.FRAME_ID)

@messaging.message_callback("takeoff")
def _command_takeoff(**kwargs):
    task_manager.add_task(time.time(), 0, animation.takeoff,
                          task_kwargs={
                              "z": client.active_client.TAKEOFF_HEIGHT,
                              "timeout": client.active_client.TAKEOFF_TIME,
                              "safe_takeoff": client.active_client.SAFE_TAKEOFF,
                              "use_leds": client.active_client.USE_LEDS,
                          }
                          )


@messaging.message_callback("land")
def _command_land(**kwargs):
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
def _command_disarm(**kwargs):
    task_manager.reset()
    task_manager.add_task(-5, 0, FlightLib.arming_wrapper,
                          task_kwargs={
                              "state": False
                          }
                          )


@messaging.message_callback("stop")
def _command_stop(**kwargs):
    task_manager.reset()


@messaging.message_callback("pause")
def _command_pause(**kwargs):
    task_manager.pause()


@messaging.message_callback("resume")
def _command_resume(**kwargs):
    task_manager.resume(time_to_start_next_task=kwargs.get("time", 0))


@messaging.message_callback("start")
def _play_animation(**kwargs):
    start_time = float(kwargs["time"])
    # Check if animation file is available
    if animation.get_id() == 'No animation':
        print("Can't start animation without animation file!")
        return

    task_manager.reset(interrupt_next_task=False)
    
    print("Start time = {}, wait for {} seconds".format(start_time, start_time-time.time()))
    # Load animation
    frames = animation.load_animation(os.path.abspath("animation.csv"),
                                        x0=client.active_client.X0 + client.active_client.X0_COMMON,
                                        y0=client.active_client.Y0 + client.active_client.Y0_COMMON,
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
        print ("Start_time")
        # Calculate start time
        start_time += start_delay
        # Arm
        task_manager.add_task(start_time, 0, FlightLib.arming_wrapper,
                            task_kwargs={
                                "state": True
                            }
                            )
        frame_time = start_time + 1.0
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
                        "timeout": client.active_client.TAKEOFF_TIME,
                        "frame_id": client.active_client.FRAME_ID,
                        "use_leds": client.active_client.USE_LEDS,
                    },
                    )
    #print(task_manager.task_queue)
        


if __name__ == "__main__":
    copter_client = CopterClient()
    task_manager = tasking.TaskManager()
    copter_client.start(task_manager)

    # ros_logging.route_logger_to_ros()
    # ros_logging.route_logger_to_ros("__main__")
    # ros_logging.route_logger_to_ros("client")
    # ros_logging.route_logger_to_ros("messaging")
