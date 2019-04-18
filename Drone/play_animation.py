import time
import csv
import rospy
import logging
from FlightLib import FlightLib
from FlightLib import LedLib

module_logger = logging.getLogger("Animation player")


animation_file_path = 'animation.csv'
USE_LEDS = True
FRAME_ID = 'aruco_map'


def takeoff(z=1.5, safe_takeoff=True, timeout=5000):
    if USE_LEDS:
        LedLib.wipe_to(255, 0, 0)
    FlightLib.takeoff(z=z, wait=True, timeout_takeoff=timeout, emergency_land=safe_takeoff,)
    LedLib.blink(0, 255, 0)


def land(descend=False):
    if USE_LEDS:
        LedLib.blink(255, 0, 0)
    FlightLib.land(descend=descend)
    if USE_LEDS:
        LedLib.off()


def animate_frame(current_frame, x0=0.0, y0=0.0, copter_frame_id=FRAME_ID):
    FlightLib.navto(current_frame['x']+x0, current_frame['y']+y0, current_frame['z'],
                    yaw=1.57, frame_id=copter_frame_id)  # TODO yaw
    if USE_LEDS:
        LedLib.fill(current_frame['red'], current_frame['green'], current_frame['blue'])


def reach_frame(current_frame, x0=0.0, y0=0.0, timeout=5000, copter_frame_id=FRAME_ID):
    FlightLib.reach_point(current_frame['x']+x0, current_frame['y']+y0, current_frame['z'],
                          yaw=1.57, timeout=timeout, frame_id=copter_frame_id)  # TODO yaw
    if USE_LEDS:
        LedLib.fill(current_frame['red'], current_frame['green'], current_frame['blue'])


def read_animation_file(filepath=animation_file_path):
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
                    'x': float(x),
                    'y': float(y),
                    'z': float(z),
                    'yaw': float(yaw),
                    'red': int(red),
                    'green': int(green),
                    'blue': int(blue),
                })
        return imported_frames


if __name__ == '__main__':
    rospy.init_node('Animation_player', anonymous=True)
    if USE_LEDS:
        LedLib.init_led()
    X0 = 0.5
    Y0 = 1.0
    frames = read_animation_file()
    rate = rospy.Rate(8)
    takeoff()
    FlightLib.reach_point(x=frames[0]['x']+X0, y=frames[0]['y']+Y0, z=frames[0]['z'], yaw=11.57)
    for frame in frames:
        animate_frame(frame, x0=X0, y0=Y0)
        rate.sleep()
    land()
    time.sleep(1)
