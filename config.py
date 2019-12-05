import os

from configobj import ConfigObj, Section
from validate import Validator


from pathlib import Path


def modify_filename(path, pattern):
    #    name = pattern.format(Path(path).stem)
    old_path, filename = os.path.split(path)
    filename = os.path.splitext(filename)[0]
    newfilename = pattern.format(filename)
    return os.path.join(old_path, newfilename)


class ConfigManager:
    def __init__(self, config=None):
        self.config = ConfigObj() if config is None else config

    def set_config(self, config):
        self.config = config

    @classmethod
    def _extract_values(cls, d):
        result = {}
        for key, val in d.items():
            if isinstance(val, dict) and val.get('__option__', False):
                if not val.get('unchanged', False):
                    result[key] = val.get('value')
            else:
                result[key] = cls._extract_values(val)
        return result

    @classmethod
    def _load_comments(cls, d, section):
        comments = {}
        inline_comments = {}

        for key, val in d.items():
            if val.get('__option__', False):
                comments[key] = val.get('comments', [])
                inline_comments[key] = val.get('inline_comment', '')
            else:
                cls._load_comments(val, section[key])
                comments[key] = ['']
                inline_comments[key] = None

        section.comments = comments
        section.inline_comments = inline_comments

    def load_from_dict(self, d, path):
        initial_comment = d.pop('initial_comment', [''])
        final_comment = d.pop('final_comment', [''])

        config = ConfigObj(infile=self._extract_values(d), indent_type='',
                           configspec=modify_filename(path, 'spec/configspec_{}.ini'))
        config.filename = path
        config.initial_comment = initial_comment
        config.final_comment = final_comment

        self._load_validate(config)
        self._load_comments(d, self.config)

    @staticmethod
    def _config_exists(path):
        return not((not path.is_file()) or path.suffix != '.ini')

    @staticmethod
    def _get_spec_path(path):
        return modify_filename(path, 'spec/configspec_{}.ini')

    def load_from_file(self, path):
        p = Path(path)
        if not self._config_exists(p):
            raise ValueError('Config file do not exist!')

        if p.name.startswith('configspec_'):
            config_path = p.parents[1].joinpath(p.name.replace('configspec_', ''))
            if self._config_exists(config_path):
                return self.load_config_and_spec(config_path)

            if p.parent.name == 'spec':
                self.generate_default_config(config_path)

            return self.load_only_spec(p)

        else:
            spec_path = Path(self._get_spec_path(p))
            if self._config_exists(spec_path):
                return self.load_config_and_spec(p)

            return self.load_only_config(p)

    def load_config_and_spec(self, path):
        path = str(path)
        self.generate_default_config(path)
        config = ConfigObj(infile=path,
                           configspec=self._get_spec_path(path))

        self._load_validate(config)

    def load_only_config(self, path: Path):
        path = str(path)
        config = ConfigObj(infile=path)
        self.set_config(config)

    def load_only_spec(self, path: Path):
        path = str(path)
        config = ConfigObj(configspec=path)
        config.filename = path.parent.joinpath(path.name.replace('configspec_', ''))

        self._load_validate(config, True)

    def _load_validate(self, config, copy=False):
        vdt = Validator()

        test = config.validate(vdt, copy=copy)
        if test != True:  # Important syntax, do no change
            raise ValueError('Some values are wrong: {}'.format(test))

        self.set_config(config)

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

    @classmethod
    def _full_dict(cls, item):
        if not isinstance(item, Section):
            return item

        d = {}
        default_values = item.default_values
        defaults = item.defaults
        comments = item.comments
        inline_comments = item.inline_comments

        for key, value in item.items():
            result = cls._full_dict(value)
            if not isinstance(result, dict):
                item_d = {'__option__': True,
                          'value': value,
                          'default': default_values.get(key, None),
                          'unchanged': key in defaults,
                          'comments': comments[key],
                          'inline_comment': inline_comments[key],
                          }
                d[key] = item_d

            else:
                d[key] = result

        return d

    @property
    def default_values(self):
        return self._get_defaults(self.config) or {}

    @property
    def unchanged_defaults(self):
        return self._get_defaults(self.config, unchanged_only=True) or {}

    @property
    def full_dict(self):
        d = self._full_dict(self.config)
        d['initial_comment'] = self.config.initial_comment
        d['final_comment'] = self.config.final_comment
        return d

    @classmethod
    def generate_default_config(cls, path):
        if cls._config_exists(path):
            return
        vdt = Validator()
        config = ConfigObj(configspec=cls._get_spec_path(path))
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
    cfg.load_from_file('Drone/config/client.ini')


    # cfg.load_config_and_spec('Drone/config/client.ini')
    # #print(cfg.config.comments)
    # #print(cfg.server_host)
    # cfg.server_host = '192.168.1.103'
    #
    # #print(cfg.get('SERVER', 'host'))
    # cfg.set('SERVER', 'host', '192.168.1.103')
    #
    # print(cfg.config.initial_comment, cfg.config.final_comment)
    #
    # # print(cfg.config)
    # # print(cfg.default_values)
    # # print(cfg.unchanged_defaults)
    #
    # # print(11111)
    import pprint
    pprint.pprint(cfg.full_dict)
    # #print(cfg.full_dict)
    #
    # #cfg.load_from_dict(cfg.full_dict, 'Drone/config/client.ini')
    # #print(cfg.config.initial_comment, cfg.config.final_comment)
    # #cfg.write()
    #

