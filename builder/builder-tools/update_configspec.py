import os
import sys
import shutil

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.realpath(os.path.join(current_dir, os.pardir, os.pardir)))

from lib.config import ConfigManager, ConfigObj
from server.modules.copter_table_models import CopterDataModel


config_path = 'temp_config/config'
spec_path = os.path.join(config_path, 'spec')
if not os.path.exists(spec_path):
    try:
        os.makedirs(spec_path)
    except OSError:
        print("Creation of the directory {} failed".format(spec_path))
    else:
        print("Successfully created the directory {}".format(spec_path))

shutil.copy("Server/config/spec/configspec_server.ini", spec_path)

config = ConfigManager()
config.load_config_and_spec(os.path.join(config_path,' server.ini'))

preset_params = config.table_presets_default
default_param = (True, 100)

default = {key: f"preset_param(default=list{preset_params.get(key, default_param)})"
           for key in CopterDataModel.columns}

cfg_server = ConfigObj('Server/config/spec/configspec_server.ini', list_values=False)
cfg_server['TABLE']['PRESETS']['DEFAULT'] = default
cfg_server.write()

print('Server configspec updated!')
shutil.rmtree('temp_config')
