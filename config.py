import collections
import os
from configobj import ConfigObj
from validate import Validator

ConfigOption = collections.namedtuple("ConfigOption", ["section", "option", "value"])


class ConfigManager:
    def __init__(self):
        self.configs = {}

    @staticmethod
    def _get_default_path(path):
        old_path, filename = os.path.split(path)
        filename = os.path.splitext(filename)[0]
        newfilename = "default_{}.ini".format(filename)
        print(os.path.join(old_path, newfilename))
        return os.path.join(old_path, newfilename)

    def load_config(self, path):  # todo maybe automatic config path
        vdt = Validator()

        default_config = ConfigObj(
            infile=self._get_default_path(path), configspec="Drone/configs/configspec_client.ini")
        default_config.validate(vdt)
        print(default_config)
        default_config.walk(self.transform)
        print(default_config.dict())
        #default_config = configparser.ConfigParser(co)
        #default_config.read(default_path)



    def create_empty_config(self, path):
        with open(path, 'w') as f:
            f.write("# Write here any configurations to replace default values \n\n")

    @staticmethod
    def getvalue(section, key):
        try:
            return section.as_int(key)
        except ValueError:
            pass
        try:
            return section.as_float(key)
        except ValueError:
            pass
        try:
            return section.as_bool(key)
        except ValueError:
            pass
        return section.get(key)

    @classmethod
    def transform(cls, section, key):
        value = cls.getvalue(section, key)
        print(value)
        section[key] = value

if __name__ == '__main__':
    cfg = ConfigManager()
    #open('Drone/default_clinet_config.ini')
    cfg.load_config('Drone/configs/clinet_config.ini')
