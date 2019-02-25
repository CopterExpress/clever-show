import time
import csv
import rospy
from FlightLib.FlightLib import FlightLib
#FlightLib.init('SingleCleverFlight')
from FlightLib.FlightLib import LedLib

animation_file_path = 'animation.csv'
frames = []
USE_LEDS = True


def takeoff():  #x, y, z
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
    FlightLib.navto(current_frame['x'], current_frame['y'], current_frame['z'], speed=current_frame['speed'])
    if USE_LEDS:
        LedLib.fill(current_frame['green'], current_frame['red'], current_frame['blue'])


def read_animation_file(filepath=animation_file_path):
    with open(filepath) as animation_file:
        csv_reader = csv.reader(
            animation_file, delimiter=',', quotechar='|'
        )
        for row in csv_reader:
            frame_number, x, y, z, yaw, red, green, blue = row
            speed = FlightLib.get_distance()
            frames.append({
                'number': int(frame_number),
                'x': float(x),
                'y': float(y),
                'z': float(z),
                'speed': float(speed),
                'red': int(red),
                'green': int(green),
                'blue': int(blue),
            })


def get_frames():
    global frames
    return frames


if __name__ == '__main__':
    rospy.init_node('Animation_player', anonymous=True)
    LedLib.init_led()

    read_animation_file()

    #first_frame = frames[0]
    #takeoff(round(float(first_frame['x']), 4), round(float(first_frame['y']), 4), round(float(first_frame['z']), 4))
    takeoff()
    #FlightLib.reach()
    for frame in frames:
        time.sleep(0.1)
        do_next_animation(frame)

    land()
    time.sleep(3)