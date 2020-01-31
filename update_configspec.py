import config
from Server.copter_table_models import CopterDataModel

cfg_server = config.ConfigObj('SERVER/config/spec/configspec_server.ini')
default = {key: 'boolean(default=True)' for key in CopterDataModel.columns}
cfg_server['TABLE']['PRESETS']['DEFAULT'] = default

cfg_server.write()
print('Server configspec updated')
