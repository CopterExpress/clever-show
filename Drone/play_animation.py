import time
import csv
import rospy
from FlightLib.FlightLib import FlightLib
#FlightLib.init('SingleCleverFlight')
from FlightLib.FlightLib import LedLib

animation_file_path = 'test_animation/test_1.csv'
frames = []
USE_LEDS = True


def takeoff():  #x, y, z
    if USE_LEDS:
        LedLib.wipe_to(0, 255, 0)
    FlightLib.takeoff1()


def land():
    if USE_LEDS:
        LedLib.blink(0, 255, 0)
    FlightLib.land1()
    if USE_LEDS:
        LedLib.off()


def do_next_animation(current_frame, x0 = 0, y0 = 0):
    FlightLib.navto(current_frame['x']+x0, current_frame['y']+y0, current_frame['z'], yaw = 1.57)
    if USE_LEDS:
        LedLib.fill(current_frame['green'], current_frame['red'], current_frame['blue'])


def read_animation_file(filepath=animation_file_path):
    with open(filepath) as animation_file:
        csv_reader = csv.reader(
            animation_file, delimiter=',', quotechar='|'
        )
        for row in csv_reader:
            frame_number, x, y, z, speed, red, green, blue = row
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
    if USE_LEDS:
        LedLib.init_led()
    X0 = 0.5
    Y0 = 1.0
    read_animation_file()
    rate = rospy.Rate(10)
    #first_frame = frames[0]
    #takeoff(round(float(first_frame['x']), 4), round(float(first_frame['y']), 4), round(float(first_frame['z']), 4))
    takeoff()
    FlightLib.reach(x=frames[0]['x']+X0, y=frames[0]['y']+Y0, z=frames[0]['z'], yaw = 1.57)
    for frame in frames:
        rate.sleep()
        do_next_animation(frame, x0 = X0, y0 = Y0)
    land()