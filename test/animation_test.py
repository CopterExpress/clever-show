import os
import sys
import shutil
from pytest import approx
import pytest

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = (os.path.dirname(os.path.realpath(__file__)))
lib_dir = os.path.realpath(os.path.join(current_dir, '../lib'))
drone_dir = os.path.realpath(os.path.join(current_dir, '../drone'))
sys.path.insert(0, lib_dir)
sys.path.insert(0, drone_dir)

from config import ConfigManager

config_path = 'animation_config/config'
spec_path = os.path.join(config_path,'spec')
if not os.path.exists(spec_path):
    try:
        os.makedirs(spec_path)
    except OSError:
        print("Creation of the directory {} failed".format(spec_path))
    else:
        print("Successfully created the directory {}".format(spec_path))

shutil.copy("../drone/config/spec/configspec_client.ini", spec_path)

config = ConfigManager()
config.load_config_and_spec(os.path.join(config_path,'client.ini'))

assert config.config_name == "client"

import modules.animation as animation

a = animation.Animation()

def test_animation_1():
    a.update_frames(config, "assets/animation_1.csv")
    assert a.id == 'basic'
    assert approx(a.original_frames[0].get_pos()) == [0,0,0]
    assert a.original_frames[0].get_color() == [204,2,0]
    assert a.original_frames[0].pose_is_valid()
    assert animation.get_numbers(a.static_begin_frames) == range(1,11)
    assert animation.get_numbers(a.takeoff_frames) == range(11,21)
    assert animation.get_numbers(a.route_frames) == range(21,31)
    assert animation.get_numbers(a.land_frames) == range(31, 41)
    assert animation.get_numbers(a.static_end_frames) == range(41, 51)
    assert animation.get_numbers(a.output_frames) == range(11,31)
    assert approx(a.static_begin_time) == 1
    assert approx(a.takeoff_time) == 1
    assert approx(a.output_frames_min_z) == 0.1
    assert approx(a.get_scaled_output(ratio=[1,2,3], offset=[4,5,6])[0].get_pos()) == [4.,5.,6.3]
    assert approx(a.get_scaled_output_min_z(ratio=[1,2,3], offset=[4,5,6])) == 6.3
    assert approx(a.get_start_point(ratio=[1,2,3], offset=[4,5,6])) == [4.,5.,6.3]

def test_animation_2():
    a.update_frames(config, "assets/animation_2.csv")
    assert a.id == 'parad'
    assert approx(a.original_frames[271].get_pos()) == [-1.00519,2.65699,0.24386]
    assert a.original_frames[271].get_color() == [7,255,0]
    assert a.original_frames[271].pose_is_valid()
    assert animation.get_numbers(a.static_begin_frames) == range(271)
    assert animation.get_numbers(a.takeoff_frames) == range(271,285)
    assert animation.get_numbers(a.route_frames) == range(285,1065)
    assert animation.get_numbers(a.land_frames) == []
    assert animation.get_numbers(a.static_end_frames) == []
    assert animation.get_numbers(a.output_frames) == range(271, 1065)
    assert approx(a.static_begin_time) == 27.1
    assert approx(a.takeoff_time) == 1.4
    assert approx(a.output_frames_min_z) == 0.24386
    assert approx(a.get_scaled_output(ratio=[1,2,3], offset=[4,5,6])[0].get_pos()) == [2.99481, 10.31398, 6.73158]
    assert approx(a.get_scaled_output_min_z(ratio=[1,2,3], offset=[4,5,6])) == 6.73158
    assert approx(a.get_start_point(ratio=[1,2,3], offset=[4,5,6])) == [2.99481, 10.31398, 6.73158]

def test_animation_3():
    a.update_frames(config, "assets/animation_3.csv")
    assert a.id == 'route'
    assert approx(a.original_frames[9].get_pos()) == [0.97783,0.0,1.0]
    assert a.original_frames[9].get_color() == [0,204,2]
    assert a.original_frames[9].pose_is_valid()
    assert animation.get_numbers(a.static_begin_frames) == []
    assert animation.get_numbers(a.takeoff_frames) == []
    assert animation.get_numbers(a.route_frames) == range(20,31)
    assert animation.get_numbers(a.land_frames) == []
    assert animation.get_numbers(a.static_end_frames) == []
    assert approx(a.static_begin_time) == 0
    assert approx(a.takeoff_time) == 0
    assert approx(a.output_frames_min_z) == 1
    assert approx(a.get_scaled_output(ratio=[1,2,3], offset=[4,5,6])[0].get_pos()) == [4,5,9]
    assert approx(a.get_scaled_output_min_z(ratio=[1,2,3], offset=[4,5,6])) == 9
    assert approx(a.get_start_point(ratio=[1,2,3], offset=[4,5,6])) == [4,5,9]

def test_animation_4():
    a.update_frames(config, "assets/animation_4.csv")
    assert a.id == 'two_drones_test'
    assert approx(a.original_frames[11].get_pos()) == [0.21774,1.4,1.0]
    assert a.original_frames[11].get_color() == [0,0,0]
    assert a.original_frames[11].pose_is_valid()
    assert animation.get_numbers(a.static_begin_frames) == range(1,12)
    assert animation.get_numbers(a.takeoff_frames) == []
    assert animation.get_numbers(a.route_frames) == range(12,141)
    assert animation.get_numbers(a.land_frames) == []
    assert animation.get_numbers(a.static_end_frames) == range(141,161)
    assert animation.get_numbers(a.output_frames) == range(12,141)
    assert approx(a.static_begin_time) == 1.1
    assert approx(a.takeoff_time) == 0
    assert approx(a.output_frames_min_z) == 1
    assert approx(a.get_scaled_output(ratio=[1,2,3], offset=[4,5,6])[0].get_pos()) == [4.21774,7.8,9]
    assert approx(a.get_scaled_output_min_z(ratio=[1,2,3], offset=[4,5,6])) == 9
    assert approx(a.get_start_point(ratio=[1,2,3], offset=[4,5,6])) == [4.21774,7.8,9]

def test_animation_no_file():
    a.update_frames(config, "zzz.csv")
    assert a.id == None
    assert a.original_frames == []
    assert a.output_frames == []
    assert animation.get_numbers(a.static_begin_frames) == []
    assert animation.get_numbers(a.takeoff_frames) == []
    assert animation.get_numbers(a.route_frames) == []
    assert animation.get_numbers(a.land_frames) == []
    assert animation.get_numbers(a.static_end_frames) == []
    assert a.static_begin_time == 0
    assert a.takeoff_time == 0
    assert a.output_frames_min_z is None
    assert a.get_scaled_output(ratio=[1,2,3], offset=[4,5,6]) == []
    assert a.get_scaled_output_min_z(ratio=[1,2,3], offset=[4,5,6]) is None
    assert a.get_start_point(ratio=[1,2,3], offset=[4,5,6]) == []


# print animation.get_numbers(a.static_begin_frames)
# print animation.get_numbers(a.takeoff_frames)
# print animation.get_numbers(a.route_frames)
# print animation.get_numbers(a.land_frames)
# print animation.get_numbers(a.static_end_frames)
# print animation.get_numbers(a.output_frames)
# print a.static_begin_time
# print a.takeoff_time
# print a.output_frames_min_z

shutil.rmtree('animation_config')
