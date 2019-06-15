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

#logging.basicConfig(  # TODO all prints as logs
#    level=logging.DEBUG, # INFO
#    format="%(asctime)s [%(name)-7.7s] [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s",
#    handlers=[
#        logging.StreamHandler(),
#    ])

logger = logging.getLogger(__name__)

#import ros_logging

class CopterClient(client.Client):
    def load_config(self):
        super(CopterClient, self).load_config()
        self.FRAME_ID = self.config.get('COPTERS', 'frame_id')
        self.TAKEOFF_HEIGHT = self.config.getfloat('COPTERS', 'takeoff_height')
        self.TAKEOFF_TIME = self.config.getfloat('COPTERS', 'takeoff_time')
        self.SAFE_TAKEOFF = self.config.getboolean('COPTERS', 'safe_takeoff')
        self.RFP_TIME = self.config.getfloat('COPTERS', 'reach_first_point_time')
        self.LAND_TIME = self.config.getfloat('COPTERS', 'land_time')

        self.X0_COMMON = self.config.getfloat('COPTERS', 'x0_common')
        self.Y0_COMMON = self.config.getfloat('COPTERS', 'y0_common')
        self.X0 = self.config.getfloat('PRIVATE', 'x0')
        self.Y0 = self.config.getfloat('PRIVATE', 'y0')
        self.USE_LEDS = self.config.getboolean('PRIVATE', 'use_leds')
        self.LED_PIN = self.config.getint('PRIVATE', 'led_pin')

    def start(self, task_manager_instance):
        client.logger.info("Init ROS node")
        rospy.init_node('Swarm_client', anonymous=True)
        if self.USE_LEDS:
            LedLib.init_led(self.LED_PIN)
        
        task_manager_instance.start()

        super(CopterClient, self).start()


@messaging.request_callback("selfcheck")
def _response_selfcheck():
    check = FlightLib.selfcheck()
    return check if check else "OK"

@messaging.request_callback("anim_id")
def _response_animation_id():
    return animation.get_id()

@messaging.request_callback("batt_voltage")
def _response_batt():
    return FlightLib.get_telemetry('body').voltage


@messaging.request_callback("cell_voltage")
def _response_cell():
    return FlightLib.get_telemetry('body').cell_voltage

@messaging.message_callback("test")
def _command_test(**kwargs):
    print("test")

@messaging.message_callback("service_restart")
def _command_service_restart(**kwargs):
    os.system("systemctl restart" + kwargs["name"])


@messaging.message_callback("led_test")
def _command_led_test(*args, **kwargs):
    LedLib.chase(255, 255, 255)
    time.sleep(2)
    LedLib.off()

@messaging.message_callback("led_fill")
def _command_emergency_led_fill(**kwargs):
    r = g = b = 0
    
    try:
        r = kwargs["red"]
    except KeyError:
        pass
    
    try:
        g = kwargs["green"]
    except KeyError:
        pass
    try:    
        b = kwargs["blue"]
    except KeyError: 
        pass
    
    LedLib.fill(r, g, b)



@messaging.message_callback("flip")
def _copter_flip():
    FlightLib.flip()

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
    task_manager.stop()


@messaging.message_callback("pause")
def _command_stop(**kwargs):
    task_manager.pause()


@messaging.message_callback("resume")
def _command_stop(**kwargs):
    task_manager.resume()


@messaging.message_callback("start")
def _play_animation(**kwargs):
    start_time = float(kwargs["time"])  # TODO

    if (animation.get_id() == 'No animation'):
        print("Can't start animation without animation file!")
        return

    print("Start time = {}, wait for {} seconds".format(start_time, time.time()-start_time))

    frames = animation.load_animation(os.path.abspath("animation.csv"),
                                      x0=client.active_client.X0 + client.active_client.X0_COMMON,
                                      y0=client.active_client.Y0 + client.active_client.Y0_COMMON,
                                      )

    task_manager.add_task(start_time, 0, animation.takeoff,
                        task_kwargs={
                            "z": client.active_client.TAKEOFF_HEIGHT,
                            "timeout": client.active_client.TAKEOFF_TIME,
                            "safe_takeoff": client.active_client.SAFE_TAKEOFF,
                            #"frame_id": client.active_client.FRAME_ID,
                            "use_leds": client.active_client.USE_LEDS,
                        }
                        )

    rfp_time = start_time + client.active_client.TAKEOFF_TIME
    task_manager.add_task(rfp_time, 0, animation.execute_frame,
                        task_kwargs={
                            "point": animation.convert_frame(frames[0])[0],
                            "color": animation.convert_frame(frames[0])[1],
                            "frame_id": client.active_client.FRAME_ID,
                            "use_leds": client.active_client.USE_LEDS,
                            "flight_func": FlightLib.reach_point,
                        }
                        )

    frame_time = rfp_time + client.active_client.RFP_TIME
    frame_delay = 0.125  # TODO from animation file
    for frame in frames:
        task_manager.add_task(frame_time, 0, animation.execute_frame,
                        task_kwargs={
                            "point": animation.convert_frame(frame)[0],
                            "color": animation.convert_frame(frame)[1],
                            "frame_id": client.active_client.FRAME_ID,
                            "use_leds": client.active_client.USE_LEDS,
                            "flight_func": FlightLib.navto,
                            }
                        )
        frame_time += frame_delay

    land_time = frame_time + client.active_client.LAND_TIME
    task_manager.add_task(land_time, 0, animation.land,
                        task_kwargs={
                            "timeout": client.active_client.TAKEOFF_TIME,
                            "frame_id": client.active_client.FRAME_ID,
                            "use_leds": client.active_client.USE_LEDS,
                        },
                        )


if __name__ == "__main__":
    
    copter_client = CopterClient()
    task_manager = tasking.TaskManager()
    
    copter_client.start(task_manager)

    #ros_logging.route_logger_to_ros()
    #ros_logging.route_logger_to_ros("__main__")
    #ros_logging.route_logger_to_ros("client")
    #ros_logging.route_logger_to_ros("messaging")

