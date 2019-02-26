import time
import csv
import rospy
from FlightLib.FlightLib import FlightLib
#FlightLib.init('SingleCleverFlight')
from FlightLib.FlightLib import LedLib

animation_file_path = 'animation.csv'
frames = []
USE_LEDS = True


def takeoff():
    if USE_LEDS:
        LedLib.wipe_to(0, 255, 0)
    FlightLib.takeoff()


def land():
    if USE_LEDS:
        LedLib.blink(0, 255, 0)
    FlightLib.land()
    if USE_LEDS:
        LedLib.off()


def do_next_animation(current_frame):
    FlightLib.navto(
        round(float(current_frame['x']), 4),
        round(float(current_frame['y']), 4),
        round(float(current_frame['z']), 4),
        round(float(current_frame['yaw']), 4),
    )
    if USE_LEDS:
        LedLib.fill(
            int(current_frame['green']),
            int(current_frame['red']),
            int(current_frame['blue'])
        )


def read_animation_file(filepath=animation_file_path):
    with open(filepath) as animation_file:
        csv_reader = csv.reader(
            animation_file, delimiter=',', quotechar='|'
        )
        for row in csv_reader:
            frame_number, x, y, z, yaw, red, green, blue,  = row
            frames.append({
                'number': frame_number,
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'red': red,
                'green': green,
                'blue': blue,
            })


def get_frames():
    global frames
    return frames


if __name__ == '__main__':
    rospy.init_node('Animation_player', anonymous=True)
    if USE_LEDS:
        LedLib.init_led()

    read_animation_file()

    takeoff()
    first_frame = frames[0]
    FlightLib.reach(round(float(first_frame['x']), 4),
                    round(float(first_frame['y']), 4),
                    round(float(first_frame['z']), 4))

    for frame in frames:
        time.sleep(0.1)
        do_next_animation(frame)

    land()
    time.sleep(3)
