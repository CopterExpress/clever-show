import os
import copy
import collections

from configobj import ConfigObj, Section, flatten_errors
from validate import Validator


def modify_filename(path, pattern):
    old_path, filename = os.path.split(path)
    filename, ext = os.path.splitext(filename)
    newfilename = pattern.format(filename) + ext
    return os.path.join(old_path, newfilename)


def parent_path(path, levels=1):
    for i in range(levels):
        path = os.path.abspath(os.path.join(path, os.pardir))
    return path


def parent_dir(path):
    return os.path.basename(os.path.normpath(path))


class ValidationError(ValueError):
    def __init__(self, message, config, errors):
        super(ValidationError, self).__init__(message)
        self.config = config
        self.errors = errors

    def flatten_errors(self):
        for entry in flatten_errors(self.config, self.errors):
            section_list, key, error = entry
            if key is not None:
                section_list.append(key)
            else:
                section_list.append('[missing section]')
            section_string = ', '.join(section_list)
            if error == False:  # Important syntax
                error = 'Missing value or section.'
            yield "[{}]: {}".format(section_string, error)


class ConfigManager:
    def __init__(self, config=None):
        self.config = ConfigObj() if config is None else config
        self.validated = False

    def get(self, section, option):
        return self.config[section][option]

    def set(self, section, option, value, write=False):
        self.config[section][option] = value
        if write:
            self.write()

    def write(self):
        self.config.write()

    def set_config(self, config):
        self.config = config
        self.validated = False

    def validate_config(self, config=None, copy_defaults=False):
        config = config or self.config
        vdt = Validator()

        test = config.validate(vdt, copy=copy_defaults, preserve_errors=True)
        if test != True:  # Important syntax, do no change
            raise ValidationError('Some config values are wrong: {}'.format(test), config, test)

        self.config = config
        self.validated = True

    @classmethod
    def _full_dict(cls, item):
        if not isinstance(item, Section):
            return item

        data = collections.OrderedDict()
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
                data[key] = item_d

            else:
                data[key] = result

        return data

    @property
    def full_dict(self):
        d = self._full_dict(self.config)
        d['initial_comment'] = self.config.initial_comment
        d['final_comment'] = self.config.final_comment
        return d

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

    @staticmethod
    def _config_exists(path):
        return os.path.isfile(path) and os.path.splitext(path)[1] == '.ini'

    @staticmethod
    def _get_spec_path(path):
        return modify_filename(path, 'spec/configspec_{}')

    @staticmethod
    def _get_config_path(path):
        filename = os.path.split(path)[1]
        return os.path.join(parent_path(path, levels=2),
                            filename.replace('configspec_', ''))

    def load_from_file(self, path):
        if not self._config_exists(path):
            raise ValueError('Config file do not exist!')

        f_path, filename = os.path.split(path)
        if filename.startswith('configspec_'):
            config_path = self._get_config_path(path)

            if self._config_exists(config_path):
                return self.load_config_and_spec(config_path)

            generate_file = parent_dir(f_path) == 'spec'
            if generate_file:
                self.generate_default_config(config_path)

            return self.load_only_spec(path, generate_file)

        else:
            spec_path = self._get_spec_path(path)
            if self._config_exists(spec_path):
                return self.load_config_and_spec(path)

            return self.load_only_config(path)

    def load_config_and_spec(self, path):
        self.generate_default_config(path)
        config = ConfigObj(infile=path,
                           configspec=self._get_spec_path(path))

        self.validate_config(config)

    def load_only_config(self, path):
        config = ConfigObj(infile=path)
        self.set_config(config)

    def load_only_spec(self, path, generate_filename=True):
        config = ConfigObj(configspec=path)
        if generate_filename:
            config.filename = self._get_config_path(path)

        self.validate_config(config, copy_defaults=True)

    @classmethod
    def generate_default_config(cls, cfg_path):
        if cls._config_exists(cfg_path):
            return False

        vdt = Validator()
        config = ConfigObj(configspec=cls._get_spec_path(cfg_path))
        config.filename = cfg_path
        config.validate(vdt, copy=True)
        config.initial_comment = ('This is generated config with default values',
                                  'Modify to configure')
        config.write()
        return True

    @classmethod
    def _extract_values(cls, d):
        result = collections.OrderedDict()
        for key, val in d.items():
            if not isinstance(val, dict):  # Pure dict option
                result[key] = val
            elif val.get('__option__', False):  # Full-dict option with params
                if not val.get('unchanged', False):
                    result[key] = val.get('value')
            else:  # Section
                result[key] = cls._extract_values(val)
        return result

    @classmethod
    def _load_comments(cls, d, section):
        comments = {}
        inline_comments = {}

        for key, val in d.items():
            if not isinstance(val, dict):  # Pure dict option
                comments[key] = []
                inline_comments[key] = None
            elif val.get('__option__', False):  # Full-dict option with params
                comment = val.get('comments', [])
                comments[key] = [] if comment == [''] else comment
                inline_comments[key] = val.get('inline_comment', None)
            else:  # Section
                cls._load_comments(val, section[key])
                comments[key] = ['']
                inline_comments[key] = None

        section.comments = comments
        section.inline_comments = inline_comments

    def load_from_dict(self, d, path=None):
        initial_comment = d.pop('initial_comment', [''])
        final_comment = d.pop('final_comment', [''])

        kwargs = {'infile': self._extract_values(d), 'indent_type': ''}
        if path is not None:
            spec_path = self._get_spec_path(path)
            if not self._config_exists(spec_path):
                spec_path = path
            if self._config_exists(spec_path):
                kwargs.update({'configspec': spec_path})

        config = ConfigObj(**kwargs)
        config.filename = path
        config.initial_comment = initial_comment
        config.final_comment = final_comment

        if path is not None:
            self.validate_config(config)
        else:
            self.set_config(config)

        self._load_comments(d, self.config)

    def merge(self, config, validate=True):
        current = copy.deepcopy(self.config)
        current.merge(config.config)
        if validate:
            self.validate_config(current)
        else:
            self.set_config(current)


if __name__ == '__main__':
    cfg = ConfigManager()
    #cfg.load_from_file('Drone/config/client.ini')
    cfg.load_from_file('Drone/config/spec/configspec_client.ini')


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
    cfg2 = ConfigManager()
    cfg2.load_from_dict({"PRIVATE": {"id": 123132}})
    pprint.pprint(cfg2.full_dict)
    cfg.merge(cfg2)
    pprint.pprint(cfg.full_dict)

    # #print(cfg.full_dict)
    #
    # #cfg.load_from_dict(cfg.full_dict, 'Drone/config/client.ini')
    # #print(cfg.config.initial_comment, cfg.config.final_comment)
    # #cfg.write()
    #

