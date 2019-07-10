import csv
import json
import time
import logging
import threading
import numpy as np

from FlightLib import FlightLib
from FlightLib import LedLib

import tasking_lib as tasking
import transformation_lib as transform

logger = logging.getLogger(__name__)

interrupt_event = threading.Event()


def requires_import(f):
    def wrapper(*args, **kwargs):
        if args[0].imported:
            return f(*args, **kwargs)
        else:
            raise RuntimeError("Frames were never imported!")

    return wrapper


class AnimationLoader(object):
    def __init__(self):
        self.anim_id = None
        self.fps = None
        self.anim_headers = {}

        self.imported = False
        self.frames = 0
        self.current_frame = 0

        self.points = np.empty((0, 3))  # to transform points
        self.colors = np.empty((0, 3), int)  # to manipulate brightness/etc
        self.yaws = np.empty((0, 0))  # to rotate
        self.extras = []  # extra functions and params for each frame

    def load_csv(self, filepath="animation.csv"):
        try:
            animation_file = open(filepath, mode="r")
        except IOError:
            logging.error("File {} can't be opened".format(filepath))
            return False
        else:
            self._read_csv(animation_file)
            self.imported = True
            return True

    def _read_csv(self, animation_file):
        with animation_file:
            csv_reader = csv.reader(animation_file, delimiter=',', quotechar='|')
            lines = list(csv_reader)
            print(lines)

            if len(lines[0]) == 1:
                header = lines.pop(0)[0]
                print("Got animation header: {}".format(header))
                jsonheader = json.loads(header)
                self.anim_id = jsonheader.pop("file", None)
                self.anim_headers = header
            else:
                print("No header in the file")
                pass

            self.frames = len(lines)
            print(lines)
            for row in lines:
                if row:
                    self._parse_row(row)

    def _parse_row(self, row):
        _frame_number, x, y, z, yaw, red, green, blue = row[:8]
        try:
            extra = row[8]
        except IndexError:
            extra = {}

        point = float(x), float(y), float(z)
        color = int(red), int(green), int(blue)
        yaw = float(yaw)

        self.points = np.append(self.points, [point], axis=0)
        self.colors = np.append(self.colors, [color], axis=0)
        self.yaws = np.append(self.yaws, [yaw])
        self.extras.append(extra)

    @requires_import
    def get_frame(self, i):
        point = tuple(self.points[i])
        color = tuple(self.colors[i])
        yaw = self.yaws[i]
        extra = self.extras[i]

        return point, color, yaw, extra

    def __getitem__(self, item):
        return self.get_frame(item)

    def __next__(self):
        try:
            val = self.get_frame(self.current_frame)
        except IndexError:
            raise StopIteration
        else:
            self.current_frame += 1
            return val

    next = __next__  # python 2 compatibility

    def offset(self, offset):
        self.points = transform.translate(self.points, np.array([offset]))

    def rotate_z(self, angle, origin=None, modify_yaw=False):
        origin = np.array([origin]) if origin is not None else np.array([[0, 0, 0]])
        self.points = transform.rotate_z(self.points, angle, origin)
        if modify_yaw:
            self.yaws += angle


if __name__ == "__main__":  # for testing animation loader
    anim = AnimationLoader()
    anim.load_csv("animation.csv")
    for fr in anim:
        print(fr)


class AnimationPlayer(object):
    def __init__(self, frames, frame_delay, interrupter=interrupt_event, common_kwargs=None):
        self.frames = list(frames)
        self.frame_time = None
        self.frame_delay = frame_delay

        self.interrupter = interrupter
        self.common_kwargs = common_kwargs if common_kwargs is not None else {}

        self._override = None

    def _execution_wrapper(self, f, *args, **kwargs):
        f(*args, **kwargs)

    def _after_frame(self):
        self.frame_time += self.frame_delay
        tasking.wait(self.frame_time, self.interrupter)

    def execute_animation(self, start_time=None, **kwargs):
        self.frame_time = start_time if start_time is not None else time.time()

        self._override = None  # additional cleanup in case of restarting same animation

        for frame in self.frames:
            if self.interrupter.is_set():
                logger.warning("Animation playing function interrupted!")
                self.interrupter.clear()
                return 'interrupted'

            self._execute_frame(frame)

            self._after_frame()

    def _execute_frame(self, frame):
        point, color, yaw, extras = frame

        if self._override is not None:
            if extras.get(self._override, False):
                return  # don't execute anything if long-term override is active
            else:
                self._override = None  # reset override when not active anymore

        for key, val in extras.keys():
            try:
                extra = extra_functions[key]
            except KeyError:
                pass
            else:
                if val:
                    if self._execute_extra(frame, key, extra):
                        return  # returns if override extra is preformed

        self._execution_wrapper(point_flight, *frame, interrupter=self.interrupter, **self.common_kwargs)

    def _execute_extra(self, frame, key, extra):
        extra_f, params = extra

        self._execution_wrapper(extra_f, frame, interrupter=self.interrupter, **self.common_kwargs)

        override = params.get("override", False)  # False is default if not found

        if params.get("oneshot", False):
            self._override = key

        return override


class TaskingAnimationPlayer(AnimationPlayer):
    def __init__(self, frames, frame_delay, task_manager=None, common_kwargs=None):
        self.task_manager = task_manager if task_manager is not None else tasking.TaskManager()

        super(TaskingAnimationPlayer, self).__init__(frames, frame_delay,
                                                     interrupter=task_manager.task_interrupt_event,
                                                     common_kwargs=common_kwargs)

    def _execution_wrapper(self, f, *args, **kwargs):
        self.task_manager.add_task(self.frame_time, 0, f,
                                   task_args=args,
                                   task_kwargs=kwargs,
                                   )

    def _after_frame(self):
        self.frame_time += self.frame_delay
        # no delay in task creating


extra_functions = {}


def extra_function(key, **params):
    def inner(f):
        extra_functions[key] = f, params

        def wrapper(*args, **kwargs):
            return f(*args, **kwargs)

        return wrapper

    return inner


@extra_function("drone_flip", override=True, oneshot=True)
def execute_flip(frame, frame_id='aruco_map', flight_kwargs=None, interrupter=interrupt_event, **kwargs):
    if flight_kwargs is None:
        flight_kwargs = {}

    FlightLib.flip(frame_id=frame_id, **flight_kwargs)


def point_flight(frame, frame_id='aruco_map', use_leds=True, use_yaw=False,
                 flight_func=FlightLib.navto, flight_kwargs=None,
                 interrupter=interrupt_event, **kwargs):
    if flight_kwargs is None:
        flight_kwargs = {}

    point, color, yaw, extras = frame
    yaw = yaw if use_yaw else float('nan')

    flight_func(*point, yaw=yaw, frame_id=frame_id, interrupter=interrupter, **flight_kwargs)
    if use_leds:
        LedLib.fill(*color)


def takeoff(z=1.5, safe_takeoff=True, frame_id='map', timeout=5.0, use_leds=True,
            interrupter=interrupt_event):
    if use_leds:
        LedLib.wipe_to(255, 0, 0, interrupter=interrupter)
    if interrupter.is_set():
        return
    result = FlightLib.takeoff(z=z, wait=False, timeout_takeoff=timeout, frame_id=frame_id, emergency_land=safe_takeoff,
                               interrupter=interrupter)
    if result == 'not armed':
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
