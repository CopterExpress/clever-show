import os
import copy
import collections

from configobj import ConfigObj, Section, flatten_errors
from validate import Validator, is_tuple, is_boolean, is_integer, is_ip_addr


def modify_filename(path, pattern):  # TODO move to core
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


def is_preset_param(value):
    parsed = is_tuple(value, min=2, max=2)
    return is_boolean(parsed[0]), is_integer(parsed[1], min=0)

def is_ip_or_local(value):
    return value if value == 'localhost' else is_ip_addr(value)


class ValidationError(ValueError):
    def __init__(self, message, config, errors):
        super(ValidationError, self).__init__(message)
        self.config = config
        self.errors = errors

    def __str__(self):
        return "{} - {}".format(self.args[0], " ".join(self.flatten_errors()))

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

        self._name_dict = {}

    def get(self, section, option):
        return self.config[section][option]

    def set(self, section, option, value, write=False):
        if section:
            self.config[section][option] = value
        else:
            self.config[option] = value
        if write:
            self.write()

    def get_chain(self, *keys):
        current = self.config
        for key in keys:
            current = current[key]
        return current

    def set_chain(self, value, *keys):  # will  create new sections!
        current = self.config
        for key in keys[:-1]:
            current = current.setdefault(key, {})
        current[keys[-1]] = value

    def write(self):
        self.config.write()

    @property
    def validated(self):
        return self.config.configspec is not None

    def set_config(self, config):
        self.config = config
        self._name_dict = self.flatten_keys(config)

    def validate_config(self, config=None, copy_defaults=False):
        config = self.config if config is None else config
        vdt = Validator({"preset_param": is_preset_param,
                         "ip": is_ip_or_local,
                         })

        test = config.validate(vdt, copy=copy_defaults, preserve_errors=True)
        if test != True:  # Important syntax, do no change
            raise ValidationError('Some config values are wrong', config, test)

        self.set_config(config)

    @classmethod
    def _full_dict(cls, item, include_defaults=False):
        if not isinstance(item, Section):
            return item

        data = collections.OrderedDict()
        default_values = item.default_values
        defaults = item.defaults
        comments = item.comments
        inline_comments = item.inline_comments

        for key, value in item.items():
            result = cls._full_dict(value, include_defaults)
            if not isinstance(result, dict):
                item_d = {'__value__': value}

                comment = comments.get(key, [])
                if comment and comment != ['']:
                    item_d.update({'comments': comment})

                inline_comment = inline_comments.get(key, None)
                if inline_comment:
                    item_d.update({'inline_comment': inline_comments})

                if include_defaults:
                    item_d.update({'default': default_values.get(key, None),
                                   'unchanged': key in defaults,
                                   })
                data[key] = item_d
            else:
                data[key] = result

        return data

    def full_dict(self, include_defaults=False):
        d = self._full_dict(self.config, include_defaults=include_defaults)
        d['initial_comment'] = self.config.initial_comment
        d['final_comment'] = self.config.final_comment
        return d

    @classmethod
    def flatten_keys(cls, d, parent_keys=(), sep='_'):
        items = {}
        for key, value in d.items():
            keys = parent_keys + (key,)
            if isinstance(value, dict):
                items.update(cls.flatten_keys(value, keys, sep=sep))
            formatted_keys = [key.lower().strip().replace(' ', sep) for key in keys]
            formatted_key = sep.join(formatted_keys)
            items.update({formatted_key: keys})
        return dict(items)

    def __getattr__(self, item):
        try:
            keys = self.__dict__['_name_dict'][item]
            return self.get_chain(*keys)
        except (ValueError, KeyError):
            try:
                return self.__dict__[item]
            except KeyError:
                print("config: KeyError with item {}".format(item))
                return None

    def __setattr__(self, key, value):
        try:
            keys = self.__dict__['_name_dict'][key]
            self.set_chain(value, *keys)
        except (ValueError, KeyError):
            self.__dict__[key] = value

    @staticmethod
    def config_exists(path):
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
        if not self.config_exists(path):
            raise ValueError('Config file does not exist!')

        f_path, filename = os.path.split(path)
        if filename.startswith('configspec_'):
            config_path = self._get_config_path(path)

            if self.config_exists(config_path):
                return self.load_config_and_spec(config_path)

            generate_file = parent_dir(f_path) == 'spec'
            if generate_file:
                self.generate_default_config(config_path)

            return self.load_only_spec(path, generate_file)

        else:
            spec_path = self._get_spec_path(path)
            if self.config_exists(spec_path):
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
        if cls.config_exists(cfg_path):
            return False

        vdt = Validator()
        config = ConfigObj(configspec=cls._get_spec_path(cfg_path))
        config.filename = cfg_path
        config.validate(vdt, copy=True)
        config.indent_type = ''
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
            elif '__value__' in val:  # Full-dict option with params
                if not val.get('unchanged', False):
                    result[key] = val.get('__value__')
            else:  # Section
                result[key] = cls._extract_values(val)
        return result

    @classmethod
    def _load_comments(cls, d, section):
        comments = section.comments
        inline_comments = section.inline_comments

        for key, val in d.items():
            if not isinstance(val, dict):  # Pure dict option
                comments[key] = []
                inline_comments[key] = None
            elif '__value__' in val:  # Full-dict option with params
                comment = val.get('comments', [])
                comments[key] = [] if comment == [''] else comment
                inline_comments[key] = val.get('inline_comment', None)
            else:  # Section
                cls._load_comments(val, section[key])
                comments[key] = ['']
                inline_comments[key] = None

        section.comments = comments
        section.inline_comments = inline_comments

    def load_from_dict(self, d, configspec=None):
        initial_comment = d.pop('initial_comment', [''])
        final_comment = d.pop('final_comment', [''])

        kwargs = {'infile': self._extract_values(d), 'indent_type': ''}
        filename = None
        if isinstance(configspec, dict):
            kwargs.update({'configspec': configspec})
        elif isinstance(configspec, str):
            spec_path = self._get_spec_path(configspec)
            if self.config_exists(spec_path):  # when 'configspec' points to configuration file and configspec exists
                kwargs.update({'configspec': spec_path})
                filename = configspec
            elif self.config_exists(configspec):  # when 'configspec' points to configspec file
                kwargs.update({'configspec': configspec})
                if parent_dir(configspec) == 'spec':
                    filename = self._get_config_path(configspec)
            else:
                raise ValueError("Configspec does not exist")

        config = ConfigObj(**kwargs)
        config.filename = filename
        config.initial_comment = initial_comment
        config.final_comment = final_comment

        if config.configspec is not None:
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
    cfg.load_from_file('drone/config/client.ini')
    # cfg.load_from_file('server/config/server.ini')
    #cfg.load_from_file('drone/config/spec/configspec_client.ini')
    print(dict(cfg.full_dict(include_defaults=True)))
    cfg.config.pop("PRIVATE", None)
    print(cfg.config)


    # cfg.load_config_and_spec('drone/config/client.ini')
    # #print(cfg.config.comments)
    # #print(cfg.server_host)
    # cfg.server_host = '192.168.1.103'
    #
    # print(cfg.get('SERVER', 'host'))
    # cfg.set('SERVER', 'host', '192.168.1.103')
    # print(cfg.get('SERVER', 'host'))
    #
    # print(cfg.config.initial_comment, cfg.config.final_comment)
    #
    # # print(cfg.config)
    # # print(cfg.default_values)
    # # print(cfg.unchanged_defaults)
    #
    # # print(11111)
    import pprint
    #pprint.pprint(cfg.full_dict)
    # cfg2 = ConfigManager()
    # #cfg2.load_from_dict({"PRIVATE": {"offset": [1, 2, 3]}}, configspec='drone/config/spec/configspec_client.ini')
    # cfg2.load_from_dict({"PRIVATE": {"id": "heh"}})
    # #pprint.pprint(cfg2.full_dict)
    # #cfg.merge(cfg2)
    # #pprint.pprint(cfg.full_dict)
    # print(cfg2.full_dict(include_defaults=True))
    #print(dict(cfg2.config.configspec))
    #print(cfg2.config.PRIVATE)
    #print(dict(ConfigManager(cfg.config.configspec).config))

    # #print(cfg.full_dict)
    #
    # #cfg.load_from_dict(cfg.full_dict, 'drone/config/client.ini')
    # #print(cfg.config.initial_comment, cfg.config.final_comment)
    # #cfg.write()
    #

