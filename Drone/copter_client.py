import os
import time
import rospy
import logging

from FlightLib import FlightLib
from FlightLib import LedLib

import client

import messaging_lib as messaging
import tasking_lib as tasking
import animation_lib as animation

import ros_logging

logger = logging.getLogger(__name__)


class CopterClient(client.Client):
    def load_config(self):
        super(CopterClient, self).load_config()
        self.FRAME_ID = self.config.get('COPTERS', 'frame_id')
        self.TAKEOFF_HEIGHT = self.config.getfloat('COPTERS', 'takeoff_height')
        self.TAKEOFF_TIME = self.config.getfloat('COPTERS', 'takeoff_time')
        self.SAFE_TAKEOFF = self.config.getboolean('COPTERS', 'safe_takeoff')
        self.RFP_TIME = self.config.getfloat('COPTERS', 'reach_first_point_time')

        self.X0_COMMON = self.config.getfloat('COPTERS', 'x0_common')
        self.Y0_COMMON = self.config.getfloat('COPTERS', 'y0_common')
        self.X0 = self.config.getfloat('PRIVATE', 'x0')
        self.Y0 = self.config.getfloat('PRIVATE', 'y0')

        self.USE_LEDS = self.config.getboolean('PRIVATE', 'use_leds')

    def start(self):
        logger.info("Init ROS node")
        rospy.init_node('Swarm_client', anonymous=True, log_level=rospy.DEBUG)
        if self.USE_LEDS:
            LedLib.init_led()

        super(CopterClient, self).start()


@messaging.request_callback("selfcheck")
def _response_selfcheck():
    check = FlightLib.selfcheck()
    return check if check else "OK"


@messaging.request_callback("batt_voltage")
def _response_batt():
    return FlightLib.get_telemetry('body').voltage


@messaging.request_callback("cell_voltage")
def _response_cell():
    return FlightLib.get_telemetry('body').cell_voltage


@messaging.message_callback("service_restart")
def _command_service_restart(*args, **kwargs):
    os.system("systemctl restart" + kwargs["name"])


@messaging.message_callback("led_test")
def _command_led_test(*args, **kwargs):
    LedLib.chase(255, 255, 255)
    time.sleep(2)
    LedLib.off()


@messaging.message_callback("takeoff")
def _command_takeoff(*args, **kwargs):
    task_manager.add_task(0, 0, animation.takeoff,
                          task_kwargs={
                              "z": client.active_client.TAKEOFF_HEIGHT,
                              "timeout": client.active_client.TAKEOFF_TIME,
                              "safe_takeoff": client.active_client.SAFE_TAKEOFF,
                              "use_leds": client.active_client.USE_LEDS,
                          }
                          )


@messaging.message_callback("land")
def _command_land(*args, **kwargs):
    task_manager.reset()
    task_manager.add_task(-5, 0, animation.land,
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
    task_manager.add_task(-10, 0, FlightLib.arming(False))


@messaging.message_callback("stop")
def _command_stop(*args, **kwargs):
    task_manager.stop()


@messaging.message_callback("pause")
def _command_stop(*args, **kwargs):
    task_manager.pause()


@messaging.message_callback("resume")
def _command_stop(*args, **kwargs):
    task_manager.resume()


@messaging.message_callback("start_animation")
def _play_animation(*args, **kwargs):
    gap = 0.25
    start_time = kwargs["start_time"]  # TODO
    frames = animation.load_animation(os.path.abspath("animation.csv"),
                                      x0=client.active_client.X0 + client.active_client.X0_COMMON,
                                      y0=client.active_client.Y0 + client.active_client.Y0_COMMON,
                                      )

    task_manager.add_task(start_time, -1, animation.takeoff,
                          task_kwargs={
                              "z": client.active_client.TAKEOFF_HEIGHT,
                              "timeout": client.active_client.TAKEOFF_TIME,
                              "safe_takeoff": client.active_client.SAFE_TAKEOFF,
                              "frame_id": client.active_client.FRAME_ID,
                              "use_leds": client.active_client.USE_LEDS,
                          }
                          )

    rfp_time = start_time + client.active_client.TAKEOFF_TIME + gap
    task_manager.add_task(rfp_time, -1, animation.execute_frame,
                          task_kwargs={
                              "point": animation.convert_frame(frames[0]),
                              "timeout": client.active_client.RFP_TIME,
                              "frame_id": client.active_client.FRAME_ID,
                              "use_leds": client.active_client.USE_LEDS,
                              "flight_func": FlightLib.reach_point,
                          }
                          )

    animation_time = rfp_time + client.active_client.RFP_TIME + gap
    frame_delay = 0.125  # TODO from animation file
    task_manager.add_task(animation_time, -1, animation.execute_animation,
                          task_kwargs={
                              "frames": frames,
                              "frame_delay": frame_delay,
                              "frame_id": client.active_client.FRAME_ID,
                              "use_leds": client.active_client.USE_LEDS,
                          }
                          )

    land_time = animation_time + len(frames)*frame_delay + gap
    task_manager.add_task(land_time, -1, animation.land,
                          task_kwargs={
                              "z": client.active_client.TAKEOFF_HEIGHT,
                              "timeout": client.active_client.TAKEOFF_TIME,
                              "frame_id": client.active_client.FRAME_ID,
                              "use_leds": client.active_client.USE_LEDS,
                          },
                          )


if __name__ == "__main__":
    # rospy.init_node('Swarm_client', anonymous=True)
    copter_client = CopterClient()
    task_manager = tasking.TaskManager()

    copter_client.start()
    ros_logging.route_logger_to_ros()
    ros_logging.route_logger_to_ros("__main__")
    ros_logging.route_logger_to_ros("client")
    ros_logging.route_logger_to_ros("messaging")
    ros_logging.route_logger_to_ros("messaging")


    task_manager.start()

