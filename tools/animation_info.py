import os
import sys
import shutil
import logging
import argparse
from tabulate import tabulate

logging.basicConfig(  # TODO all prints as logs
    level=logging.INFO,  # INFO
    stream=sys.stdout,
    format="%(asctime)s [%(name)-7.7s] [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
    ])

logger = logging.getLogger(__name__)

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = (os.path.dirname(os.path.realpath(__file__)))
root_dir = os.path.realpath(os.path.join(current_dir,'..'))
lib_dir = os.path.realpath(os.path.join(root_dir, 'lib'))
modules_dir = os.path.realpath(os.path.join(root_dir, 'drone/modules'))
sys.path.insert(0, lib_dir)
sys.path.insert(0, modules_dir)

from config import ConfigManager

def load_config(config):
    config_dir = 'animation_config/config'
    spec_path = os.path.join(config_dir,'spec')
    if not os.path.exists(spec_path):
        try:
            os.makedirs(spec_path)
        except OSError:
            logger.debug("Creation of the directory {} failed".format(spec_path))
        else:
            logger.debug("Successfully created the directory {}".format(spec_path))

    client_config_dir = os.path.realpath(os.path.join(root_dir,"drone/config"))
    client_config_path = os.path.realpath(os.path.join(client_config_dir,"client.ini"))
    client_configspec_path = os.path.realpath(os.path.join(client_config_dir,"spec/configspec_client.ini"))
    shutil.copy(client_configspec_path, spec_path)
    if os.path.exists(client_config_path):
        shutil.copy(client_config_path, config_dir)
    config.load_config_and_spec(os.path.join(config_dir,'client.ini'))

config = ConfigManager()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get animation info")
    parser.add_argument('animation', nargs='?', default='../examples/animations/basic/basic.csv',
                        help="Path to animation. Default is ../examples/animations/basic/basic.csv.")
    parser.add_argument('--config', action='store_true',
                        help="Set this option to print config info.")
    args = parser.parse_args()

    if args.config:
        print("\nLoading config copy from drone")
    load_config(config)
    if args.config:
        print("\nConfig name: {} | version: {}".format(config.config_name, config.config_version))
        print("Config animation settings:")
        for key, value in config.animation.items():
            if key == 'OUTPUT':
                print('\tOUTPUT:')
                for key, value in config.animation_output.items():
                    print("\t\t{}: {}".format(key, value))
            else:
                print("\t{}: {}".format(key, value))
        print("Config flight settings:")
        for key, value in config.flight.items():
            print("\t{}: {}".format(key, value))
    print("\nLoading animation {}".format(args.animation))
    import animation
    a = animation.Animation(args.animation, config)
    print("\nAnimation id: {} | state: {}".format(a.id, a.state))
    print("Frames separation:")
    print("\tStatic begin frames: {}".format(animation.get_numbers(a.transformed_frames[:a.takeoff_index])))
    print("\tTakeoff frames: {}".format(animation.get_numbers(a.transformed_frames[a.takeoff_index:a.route_index])))
    print("\tRoute frames: {}".format(animation.get_numbers(a.transformed_frames[a.route_index:a.land_index])))
    print("\tLand frames: {}".format(animation.get_numbers(a.transformed_frames[a.land_index:a.static_end_index])))
    print("\tStatic end frames: {}".format(animation.get_numbers(a.transformed_frames[a.static_end_index:])))
    header = animation.get_default_header()
    print("\nOutput frames on fly start action:")
    data = animation.get_table(a.output_frames, header)
    print (tabulate(data, headers=header))
    print("\nOutput frames on takeoff start action:")
    data = animation.get_table(a.output_frames_takeoff, header)
    print (tabulate(data, headers=header))

shutil.rmtree('animation_config')