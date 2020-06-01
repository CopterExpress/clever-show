import config
from server.copter_table_models import CopterDataModel

cfg_server = config.ConfigObj('SERVER/config/spec/configspec_server.ini', list_values=False)
widths = {"copter_id": 150}
default_width = 100

default = {key: f"preset_param(default=list(True, {widths.get(key, default_width)}))"
           for key in CopterDataModel.columns}

cfg_server['TABLE']['PRESETS']['DEFAULT'] = default

cfg_server.write()
print('Server configspec updated')
