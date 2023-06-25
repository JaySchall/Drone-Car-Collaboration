import configparser
import os

class SkyConfig:
    _config = configparser.ConfigParser()
    
    def __init__(self):
        try:
            self._config.read(os.path.join('res', 'config.sky'))
            print(self._config.sections())
        except Exception:
            raise ValueError("Config file is missing or was moved.")
        
    def reset_defaults(self):
        for section in self._config:
            if section == 'DEFAULT':
                continue
            for option in section:
                section[option] = self._config['DEFAULT'][option]

    def get_path(self, option):
        if option in self._config['user.paths']:
            return self._config['user.paths'][option]
        raise ValueError(f"{option} not found in paths.")
    
    def get_ssh(self, option):
        if option in self._config['user.ssh']:
            return self._config['user.ssh'][option]
        raise ValueError(f"{option} not found in ssh settings.")
    
    def get_testing(self, option):
        if option in self._config['user.testing']:
            return self._config['user.testing'][option]
        raise ValueError(f"{option} not found in testing settings.")
    
    def set_path(self, option, value):
        if option in self._config['user.paths']:
            self._config['user.paths'][option] = value
            return True
        raise ValueError(f"{option} not found in paths.")
    
    def set_ssh(self, option, value): 
        if option in self._config['user.ssh']:
            self._config['user.ssh'][option] = value
            return True
        raise ValueError(f"{option} not found in ssh settings.")
    
    def set_testing(self, option, value):
        if option in self._config['user.testing']:
            self._config['user.testing'][option] = value
            return True
        raise ValueError(f"{option} not found in testing settings.")
    
    def _write(self):
        with open('config.sky', 'w') as configfile:
            self._config.write(configfile)