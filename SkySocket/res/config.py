import configparser
import os

CONFIG_PATH = os.path.join('SkySocket', 'res', 'config.sky')

class SkyDefaults:
    DEFAULT = {
    "drone_username": "pi",
    "drone_ip": "192.168.0.2",
    "drone_password": "raspberry",
    "car_username": "pi",
    "car_ip": "192.168.0.1",
    "car_password": "raspberry",
    "video_source": "http://192.168.0.2:8080/stream?topic=/main_camera/image_raw",
    "offloading_percent": "50",
    "log_directory": "./logs",
    "results_directory": "./results",
    "drone_file_directory": "python_files/",
    "drone_file_name": "drone_client.py",
    "car_file_directory": "python_files/",
    "car_file_name": "car_server.py",
    }

class SkyConfig:
    _config = configparser.ConfigParser()
    
    def __init__(self):
        try:
            self._config.read(CONFIG_PATH)
        except Exception:
            raise ValueError("Config file is missing or was moved.")

    def get_all(self):
        ret = []
        for s in self._config.sections():
            ret.extend(self._config.items(s))
        return ret
    
    def set_all(self, new_vars):
        for s in self._config.sections():
            for option in self._config.options(s):
                self._config.set(s, option, new_vars[option])
        self._write()
        
    def _write(self):
        with open(CONFIG_PATH, 'w') as configfile:
            self._config.write(configfile)