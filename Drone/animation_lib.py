import time
import csv
import copy
import rospy
import logging
import threading

from FlightLib import FlightLib
from FlightLib import LedLib

import tasking_lib as tasking

logger = logging.getLogger(__name__)

interrupt_event = threading.Event()

anim_id = "Empty id"

# TODO refactor as class
# TODO separate code for frames transformations (e.g. for gps)

def get_id(filepath="animation.csv"):
    global anim_id
    try:
        animation_file = open(filepath)
    except IOError:
        logging.error("File {} can't be opened".format(filepath))
        anim_id = "No animation"
        return anim_id
    else:
        with animation_file:
            csv_reader = csv.reader(
                animation_file, delimiter=',', quotechar='|'
            )
            row_0 = csv_reader.next()
            if len(row_0) == 1:
                anim_id = row_0[0]
                print("Got animation_id: {}".format(anim_id))
            else:
                anim_id = "Empty id"
                print("No animation id in file")
    return anim_id

def get_start_xy(filepath="animation.csv"):
    try:
        animation_file = open(filepath)
    except IOError:
        logging.error("File {} can't be opened".format(filepath))
        anim_id = "No animation"
        return float('nan'), float('nan')
    else:
        with animation_file:
            csv_reader = csv.reader(
                animation_file, delimiter=',', quotechar='|'
            )
            try:
                row_0 = csv_reader.next()
            except:
                return float('nan'), float('nan')
            if len(row_0) == 1:
                anim_id = row_0[0]
                print("Got animation_id: {}".format(anim_id))
                try:
                    frame_number, x, y, z, yaw, red, green, blue = csv_reader.next()
                except:
                    return float('nan'), float('nan')
            else:
                anim_id = "Empty id"
                print("No animation id in file")
                try:
                    frame_number, x, y, z, yaw, red, green, blue = row_0
                except:
                    return float('nan'), float('nan')
    return float(x), float(y)


def load_animation(filepath="animation.csv", x0=0, y0=0, z0=0, x_ratio=1, y_ratio=1, z_ratio=1):
    imported_frames = []
    global anim_id
    try:
        animation_file = open(filepath)
    except IOError:
        logging.error("File {} can't be opened".format(filepath))
        anim_id = "No animation"
    else:
        with animation_file:
            csv_reader = csv.reader(
                animation_file, delimiter=',', quotechar='|'
            )
            row_0 = csv_reader.next()
            if len(row_0) == 1:
                anim_id = row_0[0]
                print("Got animation_id: {}".format(anim_id))
            else:
                print("No animation id in file")
                frame_number, x, y, z, yaw, red, green, blue = row_0
                imported_frames.append({
                    'number': int(frame_number),
                    'x': x_ratio*float(x) + x0,
                    'y': y_ratio*float(y) + y0,
                    'z': z_ratio*float(z) + z0,
                    'yaw': float(yaw),
                    'red': int(red),
                    'green': int(green),
                    'blue': int(blue),
                })
            for row in csv_reader:
                frame_number, x, y, z, yaw, red, green, blue = row
                imported_frames.append({
                    'number': int(frame_number),
                    'x': x_ratio*float(x) + x0,
                    'y': y_ratio*float(y) + y0,
                    'z': z_ratio*float(z) + z0,
                    'yaw': float(yaw),
                    'red': int(red),
                    'green': int(green),
                    'blue': int(blue),
                })
        return imported_frames

def correct_animation(frames, frame_delay=0.1, min_takeoff_height=0.5, move_delta=0.01, check_takeoff=True, check_land=True):
    corrected_frames = copy.deepcopy(frames)
    start_action = 'takeoff'
    frames_to_start = 0
    if len(corrected_frames) == 0:
        raise Exception('Nothing to correct!')
    # Check takeoff
    # If copter takes off in animation file, copter must be armed first and then all animation can be played
    if (corrected_frames[0]['z'] < min_takeoff_height) and check_takeoff:
        start_action = 'arm'
        # If the first point is low, then detect moment to arm,
        # delete all points, where copter is standing, and count time_delta
        for i in range(len(corrected_frames)-1):
            if corrected_frames[i-frames_to_start+1]['z'] - corrected_frames[i-frames_to_start]['z'] > move_delta:
                break
            del corrected_frames[i-frames_to_start]
            frames_to_start += 1
    start_delay = frames_to_start*frame_delay
    # Check Land
    # If copter lands in animation, landing points can be deleted
    if (corrected_frames[len(corrected_frames)-1]['z'] < min_takeoff_height) and check_land:
        for i in range(len(corrected_frames)-1,0,-1):
            # print i
            if abs(corrected_frames[i-1]['z'] - corrected_frames[i]['z']) < move_delta:
                break
            del corrected_frames[i]
        #for i in range(len(corrected_frames)-1,0,-1):
        #    if (abs(corrected_frames[i-1]['x'] - corrected_frames[i]['x']) > move_delta or
        #        abs(corrected_frames[i-1]['y'] - corrected_frames[i]['y']) > move_delta):
        #        break
        #    del corrected_frames[i]
    return corrected_frames, start_action, start_delay

# Needs for test
def save_corrected_animation(frames, filename="corrected_animation.csv"):
    corrected_animation = open(filename, mode='w+')
    csv_writer = csv.writer(corrected_animation, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for frame in frames:
        csv_writer.writerow([frame['number'],frame['x'], frame['y'], frame['z']])
        # print frame
    corrected_animation.close()

def convert_frame(frame):
    return ((frame['x'], frame['y'], frame['z']), (frame['red'], frame['green'], frame['blue']), frame['yaw'])


def execute_frame(point=(), color=(), yaw=float('Nan'), frame_id='aruco_map', use_leds=True,
                  flight_func=FlightLib.navto, auto_arm=False, flight_kwargs=None, interrupter=interrupt_event):
    if flight_kwargs is None:
        flight_kwargs = {}

    flight_func(*point, yaw=yaw, frame_id=frame_id, auto_arm=auto_arm, interrupter=interrupt_event, **flight_kwargs)
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


def takeoff(z=1.5, safe_takeoff=True, frame_id='map', timeout=5.0, use_leds=True,
            interrupter=interrupt_event):
    if use_leds:
        LedLib.wipe_to(255, 0, 0, interrupter=interrupter)
    if interrupter.is_set():
        return
    result = FlightLib.takeoff(height=z, timeout_takeoff=timeout, frame_id=frame_id,
                               emergency_land=safe_takeoff, interrupter=interrupter)
    if result == 'not armed' or result == 'timeout':
        raise Exception('STOP')  # Raise exception to clear task_manager if copter can't arm
    if interrupter.is_set():
        return
    if use_leds:
        LedLib.blink(0, 255, 0, wait=50, interrupter=interrupter)


def land(z=1.5, descend=False, timeout=5.0, frame_id='aruco_map', use_leds=True,
         interrupter=interrupt_event):
    if use_leds:
        LedLib.blink(255, 0, 0, interrupter=interrupter)
    FlightLib.land(z=z, descend=descend, timeout_land=timeout, frame_id_land=frame_id, interrupter=interrupter)
    if use_leds:
        LedLib.off()
