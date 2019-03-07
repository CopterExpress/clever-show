import time
import csv
import rospy
from FlightLib.FlightLib import FlightLib
#FlightLib.init('SingleCleverFlight')
from FlightLib.FlightLib import LedLib

animation_file_path = 'animation.csv'
USE_LEDS = True


def takeoff():
    if USE_LEDS:
        LedLib.wipe_to(255, 0, 0)
    FlightLib.takeoff1()  # TODO dont forget change back to takeoff


def land():
    if USE_LEDS:
        LedLib.blink(255, 0, 0)
    FlightLib.land1()
    if USE_LEDS:
        LedLib.off()


def animate_frame(current_frame, x0=0.0, y0=0.0):
    FlightLib.navto(current_frame['x']+x0, current_frame['y']+y0, current_frame['z'], yaw=1.57)
    if USE_LEDS:
        LedLib.fill(current_frame['red'], current_frame['green'], current_frame['blue'])


def read_animation_file(filepath=animation_file_path):
    imporetd_frames = []
    with open(filepath) as animation_file:
        csv_reader = csv.reader(
            animation_file, delimiter=',', quotechar='|'
        )
        for row in csv_reader:
            frame_number, x, y, z, yaw, red, green, blue = row
            imporetd_frames.append({
                'number': int(frame_number),
                'x': float(x),
                'y': float(y),
                'z': float(z),
                'yaw': float(yaw),
                'red': int(red),
                'green': int(green),
                'blue': int(blue),
            })
    return imporetd_frames


if __name__ == '__main__':
    rospy.init_node('Animation_player', anonymous=True)
    if USE_LEDS:
        LedLib.init_led()
    X0 = 0.5
    Y0 = 1.0
    frames = read_animation_file()
    rate = rospy.Rate(8)
    takeoff()
    FlightLib.reach(x=frames[0]['x']+X0, y=frames[0]['y']+Y0, z=frames[0]['z'], yaw=11.57)
    for frame in frames:
        animate_frame(frame, x0=X0, y0=Y0)
        rate.sleep()
    land()
    time.sleep(1)
