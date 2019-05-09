import time
import csv
import rospy
import logging
import threading

from FlightLib import FlightLib
from FlightLib import LedLib

import tasking_lib as tasking

logger = logging.getLogger(__name__)

interrupt_event = threading.Event()


def load_animation(filepath="animation.csv", x0=0, y0=0, z0=0):
    imported_frames = []
    try:
        animation_file = open(filepath)
    except IOError:
        logging.error("File {} can't be opened".format(filepath))
    else:
        with animation_file:
            csv_reader = csv.reader(
                animation_file, delimiter=',', quotechar='|'
            )
            for row in csv_reader:
                frame_number, x, y, z, yaw, red, green, blue = row
                imported_frames.append({
                    'number': int(frame_number),
                    'x': float(x) + x0,
                    'y': float(y) + y0,
                    'z': float(z) + z0,
                    'yaw': float(yaw),
                    'red': int(red),
                    'green': int(green),
                    'blue': int(blue),
                })
        return imported_frames


def convert_frame(frame):
    return (frame['x'], frame['y'], frame['z']), (frame["red"], frame["green"], frame["blue"]), frame["yaw"]


def execute_frame(point=(), color=(), yaw=float('Nan'), frame_id='aruco_map', use_leds=True,
                  flight_func=FlightLib.navto, flight_kwargs=None, interrupter=interrupt_event):

    if flight_kwargs is None:
        flight_kwargs = {}

    flight_func(*point, yaw=yaw, frame_id=frame_id, interrupter=interrupt_event, **flight_kwargs)
    if use_leds:
        if color:
            LedLib.fill(*color)


def execute_animation(frames, frame_delay, frame_id='aruco_map', use_leds=True, flight_func=FlightLib.navto,
                      interrupter=interrupt_event):
    next_frame_time = 0
    for frame in frames:
        if interrupter.is_set():
            logger.warning("Animation playing function interrupted!")
            interrupter.clear()
            return
        execute_frame(*convert_frame(frame), frame_id=frame_id, use_leds=use_leds, flight_func=flight_func,
                      interrupter=interrupter)

        next_frame_time += frame_delay
        tasking.wait(next_frame_time, interrupter)


def takeoff(z=1.5, safe_takeoff=True, timeout=5000, frame_id='aruco_map', use_leds=True,
            interrupter=interrupt_event):
    if use_leds:
        LedLib.wipe_to(255, 0, 0)
    FlightLib.takeoff(z=z, wait=False, timeout_takeoff=timeout, frame_id=frame_id, emergency_land=safe_takeoff,
                      interrupter=interrupter)
    if use_leds:
        LedLib.blink(0, 255, 0, wait=50)


def land(z=1.5, descend=False, timeout=5000, frame_id='aruco_map', use_leds=True,
         interrupter=interrupt_event):
    if use_leds:
        LedLib.blink(255, 0, 0)
    FlightLib.land(z=z, descend=descend, timeout_land=timeout, frame_id_land=frame_id, interrupter=interrupter)
    if use_leds:
        LedLib.off()