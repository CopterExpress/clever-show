import time
import csv
from FlightLib.FlightLib import FlightLib
#FlightLib.init('SingleCleverFlight')
from FlightLib.FlightLib import LedLib

animation_file_path = 'animation.csv'
frames = []
USE_LEDS = True


def takeoff(x, y, z):
    if USE_LEDS:
        LedLib.wipe_to(0, 255, 0)
    FlightLib.takeoff(x, y, z)


def land():
    if USE_LEDS:
        LedLib.blink(0, 255, 0)
    FlightLib.land()
    if USE_LEDS:
        LedLib.off()


def do_next_animation(current_frame):
    FlightLib.navto(
        round(float(current_frame['x']), 4), round(float(current_frame['y']), 4), round(float(current_frame['z']), 4),
        round(float(current_frame['yaw']), 4), speed=round(float(current_frame['speed']), 4)
    )
    if USE_LEDS:
        LedLib.fill(
            int(current_frame['green']), int(current_frame['red']), int(current_frame['blue'])
        )


def read_animation_file(filepath=animation_file_path):
    with open(filepath) as animation_file:
        csv_reader = csv.reader(
            animation_file, delimiter=',', quotechar='|'
        )
        for row in csv_reader:
            frame_number, x, y, z, speed, red, green, blue, yaw = row
            frames.append({
                'number': frame_number,
                'x': x,
                'y': y,
                'z': z,
                'speed': speed,
                'red': red,
                'green': green,
                'blue': blue,
                'yaw': yaw
            })


def get_frames():
    global frames
    return frames


if __name__ == '__main__':
    FlightLib.init('SingleCleverFlight')
    #LedLib.init_led()

    read_animation_file()

    first_frame = get_frames()[0]
    takeoff(round(float(first_frame['x']), 4), round(float(first_frame['y']), 4), round(float(first_frame['z']), 4))
    #FlightLib.reach()
    for frame in frames:
        time.sleep(0.1)
        do_next_animation(frame)

    land()
    time.sleep(3)