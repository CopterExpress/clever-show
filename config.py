import os

from functools import partial
from configobj import ConfigObj, Section
from validate import Validator


def modify_filename(path, pattern):
    old_path, filename = os.path.split(path)
    filename = os.path.splitext(filename)[0]
    newfilename = pattern.format(filename)
    return os.path.join(old_path, newfilename)


class ConfigManager:
    def __init__(self):
        self.config = ConfigObj()

    def load_config(self, path):
        self.generate_default_config(path)

        vdt = Validator()
        config = ConfigObj(infile=path, raise_errors=True,
                           configspec=modify_filename(path, 'spec/configspec_{}.ini'))
        test = config.validate(vdt)
        print(test)
        print(config)
        self.config = config

    def get(self, section, option):
        return self.config[section][option]

    def set(self, section, option, value, write=False):
        self.config[section][option] = value
        if write:
            self.write()

    def write(self):
        self.config.write()

    @classmethod
    def _get_defaults(cls, item, unchanged_only=False):
        if isinstance(item, Section):
            default_values = item.default_values.copy()

            if unchanged_only:
                default_list = item.defaults.copy()
                defaults = {key: default_values[key] for key in default_list if key in default_values}
            else:
                defaults = default_values

            for key, value in item.items():
                result = cls._get_defaults(value, unchanged_only=unchanged_only)
                if result is not None:
                    defaults[key] = result

            return defaults if defaults else None

    @property
    def default_values(self):
        return self._get_defaults(self.config) or {}

    @property
    def unchanged_defaults(self):
        return self._get_defaults(self.config, unchanged_only=True) or {}

    @staticmethod
    def generate_default_config(path):
        if os.path.isfile(path):
            return

        vdt = Validator()
        config = ConfigObj(configspec=modify_filename(path, 'spec/configspec_{}.ini'))
        config.filename = path
        config.validate(vdt, copy=True)
        config.initial_comment = ('This is generated config_attrs with defaults',
                                  'Modify to configure')
        config.write()

    def __getattr__(self, item):
        try:
            section, option = item.split('_', 1)
            return self.config[section.upper()][option.lower()]
        except (ValueError, KeyError):
            return self.__dict__[item]

    def __setattr__(self, key, value):
        try:
            section, option = key.split('_', 1)
            self.config[section.upper()][option.lower()] = value
        except (ValueError, KeyError):
            self.__dict__[key] = value


if __name__ == '__main__':
    cfg = ConfigManager()
    cfg.load_config('Drone/config/client.ini')

    print(cfg.server_host)
    cfg.server_host = '192.168.1.103'

    print(cfg.get('SERVER', 'host'))
    cfg.set('SERVER', 'host', '192.168.1.103')

    print(cfg.config)
    print(cfg.default_values)
    print(cfg.unchanged_defaults)
    #print(cfg.con)

