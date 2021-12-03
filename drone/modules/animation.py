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

def get_actions(frames):
    actions = []
    for frame in frames:
        actions.append(frame.action)
    return actions

def get_delays(frames):
    delays = []
    for frame in frames:
        delays.append(frame.delay)
    return delays

def get_stats(frames):
    stats = []
    for frame in frames:
        stats.append([frame.number, frame.action, frame.delay])
    return stats

def get_table(frames, header):
    table = []
    for frame in frames:
        array = []
        for name in header:
            array.append(getattr(frame, name))
        table.append(array)
    return table

def get_default_header():
    return["number", "action", "delay", "x", "y", "z", "yaw", "red", "green", "blue"]

def get_start_frame_index(frames):
    index = 0
    for frame in frames:
        if frame.action == 'stand':
            index += 1
        else:
            break
    return index

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
        self.start_time = None
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
            animation_file = open(self.filepath, 'rU')
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
                    if not row:  # skip empty rows
                        continue
                    elif len(row) == 2:
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
        self.land_index = len(self.original_frames) - 1
        frames = copy.deepcopy(self.original_frames)
        # Get takeoff index
        i = 0 # Moving index from the beginning
        self.takeoff_index = 0
        while i < len(frames):
            if moving(frames[i], frames[i+1], move_delta):
                break
            i += 1
        if i > 0:
            self.takeoff_index = i+1
        # Get route index
        i = self.takeoff_index
        self.route_index = self.takeoff_index
        while i < len(frames):
            if moving(frames[i], frames[i+1], move_delta, z = False) or (frames[i+1].z - frames[i].z <= 0):
                break
            i += 1
        if i - self.route_index > 0:
            self.route_index = i+1
        # Get static end index
        i = len(frames) - 1 # Moving index from the end
        self.static_end_index = len(self.original_frames) - 1
        while i >= 0:
            if moving(frames[i], frames[i-1], move_delta):
                break
            i -= 1
        if self.static_end_index - i > 0:
            self.static_end_index = i
        # Get land index
        self.land_index = self.static_end_index
        while i >= 0:
            if moving(frames[i], frames[i-1], move_delta, z = False) or (frames[i-1].z - frames[i].z <= 0):
                break
            i -= 1
        if self.land_index - i > 0:
            self.land_index = i

    def transform(self):
        try:
            x0, y0, z0 = numpy.array(self.config.animation_common_offset) + numpy.array(self.config.animation_private_offset)
        except (ValueError, KeyError):
            self.set_state("Can't transform animation: bad or empty config (offset in 'ANIMATION')", log_error=True)
            return
        try:
            x_ratio, y_ratio, z_ratio = self.config.animation_ratio
        except (ValueError, KeyError):
            self.set_state("Can't transform animation: bad or empty config (ratio in 'ANIMATION')", log_error=True)
            return
        self.transformed_frames = []
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
        output_frames = copy.deepcopy(self.transformed_frames)
        output_frames_takeoff = copy.deepcopy(self.transformed_frames)
        if static_begin:
            self.output_frames += output_frames[:self.takeoff_index]
            self.output_frames_takeoff += output_frames_takeoff[:self.takeoff_index]
        if takeoff:
            self.output_frames += output_frames[self.takeoff_index:self.route_index]
            if self.transformed_frames[self.takeoff_index].z >= takeoff_level:
                self.output_frames_takeoff += output_frames_takeoff[self.takeoff_index:self.route_index]
        if route:
            self.output_frames += output_frames[self.route_index:self.land_index]
            self.output_frames_takeoff += output_frames_takeoff[self.route_index:self.land_index]
        if land:
            self.output_frames += output_frames[self.land_index:self.static_end_index]
            self.output_frames_takeoff += output_frames_takeoff[self.land_index:self.static_end_index]
        if static_end:
            self.output_frames += output_frames[self.static_end_index:]
            self.output_frames_takeoff += output_frames_takeoff[self.static_end_index:]
        if self.output_frames:
            self.output_frames_min_z = min(self.output_frames, key = lambda p: p.z).z
        if self.output_frames_takeoff:
            self.output_frames_takeoff_min_z = min(self.output_frames_takeoff, key = lambda p: p.z).z

    def mark_flight(self):
        if not self.output_frames:
            return
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
                frame.delay = land_delay
                self.output_frames[i].action = 'land'
                self.output_frames.insert(i, frame)
                break
            i -= 1
        i = len(self.output_frames_takeoff) - 1
        # add land frame to output_frames_takeoff
        while i >=0:
            if self.output_frames_takeoff[i].action == 'fly':
                frame = copy.deepcopy(self.output_frames_takeoff[i])
                frame.delay = land_delay
                self.output_frames_takeoff[i].action = 'land'
                self.output_frames_takeoff.insert(i, frame)
                break
            i -= 1
        self.start_frame_index = get_start_frame_index(self.output_frames)
        self.start_time = get_duration(self.output_frames[:self.start_frame_index])

    def on_animation_update(self, filepath="animation.csv", config=None):
        if config is None:
            config = self.config
        self.reset(filepath, config)
        self.load()
        if self.original_frames:
            self.on_config_update(self.config)

    def on_config_update(self, config):
        self.config = config
        self.transform()
        self.mark_stand_frames()
        self.apply_flags()
        self.mark_flight()

    def get_start_frame(self, action):
        try:
            if action == 'fly':
                return self.output_frames[self.start_frame_index]
            if action == 'takeoff':
                return self.output_frames_takeoff[self.start_frame_index]
        except IndexError:
            return None

    def get_output_frames(self, action):
        if action == 'fly':
            return self.output_frames
        if action == 'takeoff':
            return self.output_frames_takeoff
        return []

    def get_min_z(self, action):
        if action == 'fly':
            return self.output_frames_min_z
        if action == 'takeoff':
            return self.output_frames_takeoff_min_z
        return []

    def get_start_action(self, current_height=0, state="STANDBY", tolerance = 0.2):
        # Check output frames
        if not self.output_frames:
            return 'error: empty output frames'
        # Check current_height
        if self.config.animation_check_ground:
            if math.isnan(current_height):
                return 'error: bad copter height'
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
            if self.output_frames[0].z > takeoff_level:
                start_action = 'takeoff'
            else:
                start_action = 'fly'
        elif start_action in ('takeoff', 'fly'):
            pass
        else:
            return 'error in [ANIMATION] start_action parameter'
        # Check that bottom point of animation is higher than ground level
        if self.config.animation_check_ground:
            try:
                ground_level = self.config.animation_ground_level
                if ground_level == 'current':
                    ground_level = current_height
                ground_level = float(ground_level)
            except (ValueError, KeyError):
                return 'error in [ANIMATION] ground_level parameter'
            if state != "ACTIVE" and ground_level - tolerance > self.get_min_z(start_action):
                return 'error: animation is lower than ground level for {:.2f}m'.format(
                    ground_level - self.output_frames_min_z
                )
        return start_action

try:
    def execute_frame(frame, config, interrupter=interrupt_event):
        auto_arm = False
        use_leds = config.led_use
        frame_id = config.flight_frame_id
        if frame.action == 'takeoff':
            use_leds = use_leds & config.led_takeoff_indication
            takeoff(z=config.flight_takeoff_height, frame_id=frame_id, timeout=config.flight_takeoff_time, use_leds=use_leds, interrupter=interrupter)
            return
        if frame.action == 'land':
            use_leds = use_leds & config.led_land_indication
            land(frame_id=frame_id, timeout=config.flight_land_timeout, use_leds=use_leds, interrupter=interrupter)
            return
        if frame.action in ('fly', 'arm'):
            if frame.action == 'arm':
                auto_arm = True
            if frame.pose_is_valid():
                flight.navto(x=frame.x, y=frame.y, z=frame.z, yaw=frame.yaw, frame_id=frame_id, auto_arm=auto_arm, interrupter=interrupter)
            else:
                logger.error("Frame pose is not valid for flying")
        if use_leds:
            try:
                red, green, blue = frame.get_color()
            except ValueError:
                logger.error("Can't get frame color!")
            else:
                led.set_effect(r=red, g=green, b=blue)

    def turn_off_led(interrupter=interrupt_event):
        led.set_effect(r=0, g=0, b=0)

    def takeoff(z=1.5, safe_takeoff=False, frame_id='map', timeout=5.0, use_leds=True,
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
        led.set_effect(r=0, g=0, b=0)
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
