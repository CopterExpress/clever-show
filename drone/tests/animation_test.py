import os
import sys
import shutil
from pytest import approx
import pytest
import logging

logging.basicConfig(  # TODO all prints as logs
    level=logging.DEBUG,  # INFO
    stream=sys.stdout,
    format="%(asctime)s [%(name)-7.7s] [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
    ])

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = (os.path.dirname(os.path.realpath(__file__)))
root_dir = os.path.realpath(os.path.join(current_dir,'../..'))
lib_dir = os.path.realpath(os.path.join(root_dir, 'lib'))
modules_dir = os.path.realpath(os.path.join(root_dir, 'drone/modules'))
sys.path.insert(0, lib_dir)
sys.path.insert(0, modules_dir)

# print("PATH: {}".format(sys.path))

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

configspec_path = os.path.realpath(os.path.join(root_dir,"drone/config/spec/configspec_client.ini"))
shutil.copy(configspec_path, spec_path)

config = ConfigManager()
config.load_config_and_spec(os.path.join(config_path,'client.ini'))

assert config.config_name == "client"

import animation

assets_dir = os.path.realpath(os.path.join(root_dir, 'drone/tests/assets'))

def test_animation_1_2():
    a = animation.Animation(os.path.join(assets_dir, 'animation_1.csv'), config)
    assert a.id == 'basic'
    assert a.state == "OK"
    assert approx(a.original_frames[0].get_pos()) == [0,0,0]
    assert a.original_frames[0].get_color() == [204,2,0]
    assert a.original_frames[0].pose_is_valid()
    assert a.takeoff_index == 10
    assert a.route_index == 20
    assert a.land_index == 29
    assert a.static_end_index == 39
    assert a.start_frame_index == a.takeoff_index
    assert approx(a.start_time) == 1
    assert a.output_frames[a.start_frame_index].action == 'arm'
    assert a.output_frames_takeoff[a.start_frame_index].action == 'takeoff'
    assert approx(a.output_frames_min_z) == 0
    assert a.get_start_action() == 'fly'
    config.set('ANIMATION', 'ratio', [1,2,3])
    config.set('ANIMATION', 'common_offset', [4,5,6])
    a.on_config_update(config)
    assert approx(a.output_frames[0].get_pos()) == [4.,5.,6.]
    assert approx(a.output_frames_min_z) == 6.
    assert approx(a.get_start_frame('fly').get_pos()) == [4.,5.,6.]
    assert a.get_start_action() == 'takeoff'
    config.set('ANIMATION', 'ratio', [1,1,1])
    config.set('ANIMATION', 'common_offset', [0,0,0])

    a.on_animation_update(os.path.join(assets_dir, 'animation_2.csv'), config)
    assert a.id == 'parad'
    assert a.state == "OK"
    assert approx(a.original_frames[271].get_pos()) == [-1.00519,2.65699,0.24386]
    assert a.original_frames[271].get_color() == [7,255,0]
    assert a.original_frames[271].pose_is_valid()
    assert a.takeoff_index == 271
    assert a.route_index == 285
    assert a.land_index == 1064
    assert a.static_end_index == 1064
    assert a.start_frame_index == a.takeoff_index
    assert approx(a.start_time) == 27.1
    assert a.output_frames[a.start_frame_index].action == 'arm'
    assert a.output_frames_takeoff[a.start_frame_index].action == 'takeoff'
    assert approx(a.output_frames_min_z) == 0.21
    assert a.get_start_action() == 'fly'
    config.set('ANIMATION', 'ratio', [1,2,3])
    config.set('ANIMATION', 'common_offset', [4,5,6])
    a.on_config_update(config)
    assert approx(a.output_frames[0].get_pos()) == [2.99481, 10.31398, 6.63]
    assert approx(a.output_frames_min_z) == 6.63
    assert approx(a.get_start_frame('fly').get_pos()) == [2.99481, 10.31398, 6.63]
    assert a.get_start_action() == 'takeoff'
    config.set('ANIMATION', 'ratio', [1,1,1])
    config.set('ANIMATION', 'common_offset', [0,0,0])

def test_animation_3():
    a = animation.Animation(os.path.join(assets_dir, 'animation_3.csv'), config)
    assert a.id == 'route'
    assert a.state == "OK"
    assert approx(a.original_frames[9].get_pos()) == [0.97783,0.0,1.0]
    assert a.original_frames[9].get_color() == [0,204,2]
    assert a.original_frames[9].pose_is_valid()
    assert a.takeoff_index == 0
    assert a.route_index == 0
    assert a.land_index == 10
    assert a.static_end_index == 10
    assert a.start_frame_index == a.takeoff_index
    assert approx(a.start_time) == 0
    assert a.output_frames[a.start_frame_index].action == 'arm'
    assert a.output_frames_takeoff[a.start_frame_index].action == 'takeoff'
    assert approx(a.output_frames_min_z) == 1
    assert a.get_start_action() == 'takeoff'
    config.set('ANIMATION', 'ratio', [1,2,3])
    config.set('ANIMATION', 'common_offset', [4,5,6])
    a.on_config_update(config)
    assert approx(a.output_frames[0].get_pos()) == [4,5,9]
    assert approx(a.output_frames_min_z) == 9
    assert approx(a.get_start_frame('fly').get_pos()) == [4,5,9]
    assert a.get_start_action() == 'takeoff'
    config.set('ANIMATION', 'ratio', [1,1,1])
    config.set('ANIMATION', 'common_offset', [0,0,0])

def test_animation_4():
    a = animation.Animation(os.path.join(assets_dir, 'animation_4.csv'), config)
    assert a.id == 'two_drones_test'
    assert a.state == "OK"
    assert approx(a.original_frames[11].get_pos()) == [0.21774,1.4,1.0]
    assert a.original_frames[11].get_color() == [0,0,0]
    assert a.original_frames[11].pose_is_valid()
    assert a.takeoff_index == 11
    assert a.route_index == 11
    assert a.land_index == 139
    assert a.static_end_index == 139
    assert a.start_frame_index == 0
    assert approx(a.start_time) == 0
    assert a.output_frames[a.start_frame_index].action == 'arm'
    assert a.output_frames_takeoff[a.start_frame_index].action == 'takeoff'
    assert approx(a.output_frames_min_z) == 1
    assert a.get_start_action() == 'takeoff'
    config.set('ANIMATION', 'ratio', [1,2,3])
    config.set('ANIMATION', 'common_offset', [4,5,6])
    a.on_config_update(config)
    assert approx(a.output_frames[0].get_pos()) == [4.2,7.8,9]
    assert approx(a.output_frames_min_z) == 9
    assert approx(a.get_start_frame('fly').get_pos()) == [4.2,7.8,9]
    assert a.get_start_action() == 'takeoff'
    config.set('ANIMATION', 'ratio', [1,1,1])
    config.set('ANIMATION', 'common_offset', [0,0,0])

def test_animation_no_file():
    a = animation.Animation('zzz.csv', config)
    assert a.id == None
    assert a.state != "OK"
    assert a.original_frames == []
    assert a.output_frames == []
    assert a.output_frames_min_z is None

shutil.rmtree('animation_config')
