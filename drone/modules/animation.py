import os
import csv
import copy
import math
import time
import numpy
import logging
import threading

logger = logging.getLogger(__name__)

# Import flight control
try:
    import modules.flight as flight
except ImportError:
    logger.debug("Can't import flight control module!")

# Import led control
try:
    import modules.led as led
except ImportError:
    logger.debug("Can't import led control module!")

interrupt_event = threading.Event()

def moving(f1, f2, delta, x=True, y=True, z=True):
    return ((abs(f1.x - f2.x) > delta) and x
        or  (abs(f1.y - f2.y) > delta) and y
        or  (abs(f1.z - f2.z) > delta) and z)

def get_numbers(frames):
    numbers = []
    for frame in frames:
        numbers.append(frame.number)
    return numbers

def get_duration(frames):
    duration = 0
    for frame in frames:
        duration += frame.delay
    return duration


class Frame(object):
    params_dict = {
        "number": None,
        "action": 'fly',    # fly, arm, land, stand
        "x": None,
        "y": None,
        "z": None,
        "yaw": None,
        "red": None,
        "green": None,
        "blue": None,
        "delay": 0,
    }
    def __init__(self, csv_row=None, delay=None):
        for key, value in self.params_dict.items():
            setattr(self, key, value)
        if csv_row:
            self.load_csv_row(csv_row)
        if delay:
            self.delay = delay

    def load_csv_row(self, csv_row):
        number, x, y, z, yaw, red, green, blue = csv_row
        self.number = int(number)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.yaw = float(yaw)
        self.red = int(red)
        self.green = int(green)
        self.blue = int(blue)

    def get_pos(self):
        if None in [self.x, self.y, self.z]:
            return []
        else:
            return [self.x, self.y, self.z]

    def get_color(self):
        if None in [self.red, self.green, self.blue]:
            return []
        else:
            return [self.red, self.green, self.blue]

    def set_yaw(self, yaw):
        if yaw != "animation":
            self.yaw = math.radians(float(yaw))

    def pose_is_valid(self):
        return self.get_pos() and (self.yaw is not None)

class Animation(object):
    # filepath - path to csv animation file, config - config 'ANIMATION' section (dictionary)
    def __init__(self, filepath="animation.csv", config=None):
        self.reset(filepath, config)
        if config is not None:
            self.on_animation_update(self.filepath)

    def reset(self, filepath, config):
        self.id = None
        self.original_frames = []
        self.transformed_frames = []
        self.static_begin_index = 0
        self.takeoff_index = 0
        self.route_index = 0
        self.land_index = 0
        self.static_end_index = 0
        self.output_frames = []
        self.output_frames_min_z = None
        self.output_frames_takeoff = []
        self.output_frames_takeoff_min_z = None
        self.filepath = filepath
        self.config = config
        self.state = None

    def set_state(self, state, log_error=False):
        self.state = state
        if log_error:
            logger.error(state)

    def load(self):
        self.state = "OK"
        try:
            delay = self.config.animation_frame_delay
        except (TypeError, KeyError):
            self.set_state("Bad animation delay from config 'ANIMATION' section", log_error=True)
            return
        try:
            animation_file = open(self.filepath)
        except IOError:
            self.set_state("File {} can't be opened".format(self.filepath), log_error=True)
        else:
            with animation_file:
                current_frame_delay = delay
                csv_reader = csv.reader(
                    animation_file, delimiter=',', quotechar='|'
                )
                try:
                    row_0 = csv_reader.next()
                except StopIteration:
                    self.set_state("Animation file is empty", log_error=True)
                    return
                if len(row_0) == 1:
                    self.id = row_0[0]
                    logger.debug("Got animation_id: {}".format(self.id))
                elif len(row_0) == 2:
                    try:
                        current_frame_delay = float(row_0[1])
                        logger.debug("Got new frame delay: {}".format(current_frame_delay))
                    except ValueError as e:
                        self.set_state("Can't parse delay row in csv file. {}".format(e), log_error=True)
                        return
                else:
                    logger.debug("No animation id in file")
                    self.id = "No animation id"
                    try:
                        frame = Frame(row_0, current_frame_delay)
                    except ValueError as e:
                        self.set_state("Can't parse frame row in csv file. {}".format(e), log_error=True)
                        return
                    self.original_frames.append(frame)
                for row in csv_reader:
                    if len(row) == 2:
                        try:
                            current_frame_delay = float(row_0[1])
                            logger.debug("Got new frame delay: {}".format(current_frame_delay))
                        except ValueError as e:
                            self.set_state("Can't parse delay row in csv file. {}".format(e), log_error=True)
                            return
                    else:
                        try:
                            frame = Frame(row, current_frame_delay)
                        except ValueError as e:
                            self.set_state("Can't parse frame row in csv file. {}".format(e), log_error=True)
                            return
                        self.original_frames.append(frame)
            self.set_yaw()
            if self.state == "OK":
                if self.original_frames:
                    self.split()
                else:
                    self.set_state("No frames loaded!", log_error=True)

    def set_yaw(self):
        try:
            yaw = self.config.animation_yaw
        except (TypeError, KeyError) as e:
            self.set_state("Can't set yaw from config 'ANIMATION' section. {}".format(e), log_error=True)
            return
        for frame in self.original_frames:
            try:
                frame.set_yaw(yaw)
            except ValueError as e:
                self.set_state("Can't set yaw from config 'ANIMATION' section. {}".format(e), log_error=True)
                return

    def split(self, move_delta=0.01):
        '''
        Split animation into 5 parts: static_begin, takeoff, route, land, static_end
            * static_begin and static_end are chains of frames in the beginning and the end of animation,
                where the drone doesn't move
            * takeoff and land are chains of frames after and before static frames of animation,
                where the drone doesn't move in xy plane, and it's z coordinate only increases or decreases, respectively.
            * route is the rest of the animation
        '''
        if len(self.original_frames) == 0:
            return
        frames = copy.deepcopy(self.original_frames)
        # Get takeoff index
        i = 0 # Moving index from the beginning
        while i < len(frames):
            if moving(frames[i], frames[i+1], move_delta):
                break
            i += 1
        self.takeoff_index = i
        # Get route index
        while i < len(frames):
            if moving(frames[i], frames[i+1], move_delta, z = False) or (frames[i+1].z - frames[i].z <= 0):
                break
            i += 1
        self.route_index = i
        # Get static end index
        i = len(frames) - 1 # Moving index from the end
        while i >= 0:
            if moving(frames[i], frames[i-1], move_delta):
                break
            i -= 1
        self.static_end_index = i
        # Get land index
        while i >= 0:
            if moving(frames[i], frames[i-1], move_delta, z = False) or (frames[i-1].z - frames[i].z <= 0):
                break
            i -= 1
        self.land_index = i

    def transform(self):
        try:
            x0, y0, z0 = self.config.animation_offset
        except (ValueError, KeyError):
            self.set_state("Can't transform animation: bad or empty config (offset in 'ANIMATION')", log_error=True)
            return
        try:
            x_ratio, y_ratio, z_ratio = self.config.animation_ratio
        except (ValueError, KeyError):
            self.set_state("Can't transform animation: bad or empty config (ratio in 'ANIMATION')", log_error=True)
            return
        self.transformed_frames = copy.deepcopy(self.original_frames)
        for frame in self.transformed_frames:
            frame.x = x_ratio*frame.x + x0
            frame.y = y_ratio*frame.y + y0
            frame.z = z_ratio*frame.z + z0

    def mark_stand_frames(self):
        if not self.transformed_frames:
            return
        try:
            takeoff_level = self.config.animation_takeoff_level
        except (ValueError, KeyError):
            self.set_state("Can't set frame actions: bad or empty config (takeoff_level in 'ANIMATION')", log_error=True)
            return
        static_begin_action = "fly"
        static_end_action = "fly"

        # Check first frame
        if self.transformed_frames[0].z < takeoff_level:
            static_begin_action = "stand"
        # Set action for static_begin frames
        for frame in self.transformed_frames[:self.takeoff_index]:
            frame.action = static_begin_action

        # Check last frame
        if self.transformed_frames[-1].z < takeoff_level:
            static_end_action = "stand"
        # Set action for static_end frames
        for frame in self.transformed_frames[self.static_end_index:]:
            frame.action = static_end_action

    def apply_flags(self):
        self.output_frames = []
        self.output_frames_takeoff = []
        if not self.transformed_frames:
            return
        try:
            static_begin = self.config.animation_output_static_begin
            takeoff = self.config.animation_output_takeoff
            route = self.config.animation_output_route
            land = self.config.animation_output_land
            static_end = self.config.animation_output_static_end
        except (ValueError, KeyError):
            self.set_state("Can't get output flags: bad or empty config ('OUTPUT' in 'ANIMATION')", log_error=True)
            return
        try:
            takeoff_level = self.config.animation_takeoff_level
        except (ValueError, KeyError):
            self.set_state("Can't set frame actions: bad or empty config (takeoff_level in 'ANIMATION')", log_error=True)
            return
        if static_begin:
            self.output_frames += self.transformed_frames[:self.takeoff_index]
            self.output_frames_takeoff += self.transformed_frames[:self.takeoff_index]
        if takeoff:
            self.output_frames += self.transformed_frames[self.takeoff_index:self.route_index]
            if self.transformed_frames[self.takeoff_index].z >= takeoff_level:
                self.output_frames_takeoff += self.transformed_frames[self.takeoff_index:self.route_index]
        if route:
            self.output_frames += self.transformed_frames[self.route_index:self.land_index]
            self.output_frames_takeoff += self.transformed_frames[self.route_index:self.land_index]
        if land:
            self.output_frames += self.transformed_frames[self.land_index:self.static_end_index]
            self.output_frames_takeoff += self.transformed_frames[self.land_index:self.static_end_index]
        if static_end:
            self.output_frames += self.transformed_frames[self.static_end_index:]
            self.output_frames_takeoff += self.transformed_frames[self.static_end_index:]
        if self.output_frames:
            self.output_frames_min_z = min(self.output_frames, key = lambda p: p.z).z
        if self.output_frames_takeoff:
            self.output_frames_takeoff_min_z = min(self.output_frames_takeoff, key = lambda p: p.z).z

    def mark_flight(self):
        try:
            arming_time = self.config.flight_arming_time
            takeoff_time = self.config.flight_takeoff_time
            rfp_time = self.config.flight_reach_first_point_time
            land_delay = self.config.flight_land_delay
        except (ValueError, KeyError):
            self.set_state("Can't mark flight: bad or empty config ('FLIGHT' section)", log_error=True)
            return
        # add arm frame to output_frames
        i = 0
        while i < len(self.output_frames):
            if self.output_frames[i].action == 'fly':
                frame = copy.deepcopy(self.output_frames[i])
                frame.action = 'arm'
                frame.delay = arming_time
                self.output_frames.insert(i, frame)
                break
            i += 1
        # add takeoff frame to output_frames_takeoff
        i = 0
        while i < len(self.output_frames_takeoff):
            if self.output_frames_takeoff[i].action == 'fly':
                # set first fly frame action to reach point
                self.output_frames_takeoff[i].action = 'reach'
                self.output_frames_takeoff[i].delay = rfp_time
                # add takeoff action before reach point
                frame = copy.deepcopy(self.output_frames_takeoff[i])
                frame.action = 'takeoff'
                frame.delay = takeoff_time
                self.output_frames_takeoff.insert(i, frame)
                break
            i += 1
        # add land frame to output_frames
        i = len(self.output_frames) - 1
        while i >=0:
            if self.output_frames[i].action == 'fly':
                frame = copy.deepcopy(self.output_frames[i])
                frame.action = 'land'
                self.output_frames[i].delay = land_delay
                self.output_frames.insert(i, frame)
        i = len(self.output_frames_takeoff) - 1
        # add land frame to output_frames_takeoff
        while i >=0:
            if self.output_frames_takeoff[i].action == 'fly':
                frame = copy.deepcopy(self.output_frames_takeoff[i])
                frame.action = 'land'
                self.output_frames_takeoff[i].delay = land_delay
                self.output_frames_takeoff.insert(i, frame)

    def on_animation_update(self, filepath="animation.csv"):
        self.filepath = filepath
        self.load()
        if self.original_frames:
            self.on_config_update(self.config)

    def on_config_update(self, config):
        self.config = config
        self.transform()
        self.mark_stand_frames
        self.apply_flags()
        self.mark_flight()

    def get_start_frame(self):
        if not self.output_frames:
            return Frame()
        return self.output_frames[0]

    def get_start_action(self, current_height, state="STANDBY", tolerance = 0.2):
        # Check output frames
        if not self.output_frames:
            return 'error: empty output frames'
        # Check current_height
        if math.isnan(current_height):
            return 'error: bad copter height'
        # Check that bottom point of animation is higher than ground level
        try:
            ground_level = self.config.animation_ground_level
            if ground_level == 'current':
                ground_level = current_height
            ground_level = float(ground_level)
        except (ValueError, KeyError):
            return 'error in [ANIMATION] ground_level parameter'
        if state != "ACTIVE" and ground_level - tolerance > self.output_frames_min_z:
            return 'error: animation is lower than ground level for {:.2f}m'.format(
                ground_level - self.output_frames_min_z
            )
        # Select start action
        try:
            start_action = self.config.animation_start_action
        except (ValueError, KeyError):
            return 'error in [ANIMATION] start_action'
        try:
            takeoff_level = self.config.animation_takeoff_level
        except (ValueError, KeyError):
            return 'error in [ANIMATION] takeoff_level'
        if start_action == 'auto':
            if self.get_start_frame().z > takeoff_level:
                return 'takeoff'
            else:
                return 'fly'
        elif start_action in ('takeoff', 'fly'):
            return start_action
        else:
            return 'error in [ANIMATION] start_action parameter'

try:
    def execute_frame(frame, frame_id='map', use_leds=True, auto_arm=False, interrupter=interrupt_event, flight_kwargs=None):
        if flight_kwargs is None:
            flight_kwargs = {}
        if frame.action in ('fly', 'arm'):
            if frame.action == 'arm':
                auto_arm = True
            if frame.pose_is_valid():
                flight_func(x=frame.x, y=frame.y, z=frame.z, yaw=frame.yaw, frame_id=frame_id, auto_arm=auto_arm, interrupter=interrupt_event, **flight_kwargs)
            else:
                logger.error("Frame pose is not valid for flying")
        if use_leds:
            try:
                red, green, blue = frame.get_color()
            except ValueError:
                logger.error("Can't get frame color!")
            else:
                led.set_effect(r=red, g=green, b=blue)

    def takeoff(z=1.5, safe_takeoff=True, frame_id='map', timeout=5.0, use_leds=True,
                interrupter=interrupt_event):
        if use_leds:
            led.set_effect(effect='wipe', r=255, g=0, b=0)
        result = flight.takeoff(height=z, timeout_takeoff=timeout, frame_id=frame_id,
                                emergency_land=safe_takeoff, interrupter=interrupter)
        if result == 'not armed':
            raise Exception('STOP')  # Raise exception to clear task_manager if copter can't arm
        if use_leds:
            led.set_effect(effect='blink_fast', r=0, g=255, b=0)


    def land(z=1.5, descend=False, timeout=5.0, frame_id='map', use_leds=True,
            interrupter=interrupt_event):
        if use_leds:
            led.set_effect(effect='blink_fast', r=255, g=0, b=0)
        flight.land(z=z, descend=descend, timeout_land=timeout, frame_id_land=frame_id, interrupter=interrupter)
        if use_leds:
            while (flight.get_telemetry_locked().armed):
                if interrupter.is_set():
                    break
                time.sleep(0.5)
            led.set_effect(r=0, g=0, b=0)

except NameError:
    print("Can't create flying functions")
