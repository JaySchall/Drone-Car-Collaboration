"""
Files and values relating to GUI configuration.

Values:
    CONFIG_PATH

Classes:
    SkyDefaults
    _SkyConfig
    SkyVariables
"""

import configparser
import os

from kivy.event import EventDispatcher

CONFIG_PATH = os.path.join("SkySocket", "res", "config.sky")


class SkyDefaults:
    """Holds a dictionary of the default values for 'reset to defaults'."""

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


class _SkyConfig:
    """
    Contains user setting values as well as read/write access to config file.

    Attributes:
        _config: The ConfigParser object.
    """
    _config = configparser.ConfigParser()
    
    def __init__(self):
        """Reads in the config file. Raises an error if file not found."""

        try:
            self._config.read(CONFIG_PATH)
        except Exception:
            raise ValueError("Config file is missing or was moved.")

    def get_all(self):
        """
        Getter for all user settings.
        
        Returns:
            A list of tuples containing name, value pairs.
        """

        ret = []
        for s in self._config.sections():
            ret.extend(self._config.items(s))
        return ret
    
    def set_all(self, new_vars):
        """
        Setter for all user settings (Expects all options to be set).

        Arg:
            new_vars: A dictionary of name, value pairs
        """

        for s in self._config.sections():
            for option in self._config.options(s):
                self._config.set(s, option, new_vars[option])
        self._write()
        
    def _write(self):
        """Writes options to config file."""

        with open(CONFIG_PATH, "w") as configfile:
            self._config.write(configfile)


class SkyVariables(_SkyConfig, EventDispatcher):
    """
    Provides access for the GUI to user configuration values.
    
    Attributes:
        variables: A dictionary of name, value pairs.
    """

    variables = {}
    __getattr__ = variables.get
    __setattr__ = variables.__setitem__
    __getitem__ = variables.get
    
    def __init__(self):
        """Initializes the _SkyConfig parser and retrieves values."""

        super().__init__()
        self._get_all()
    
    def _get_all(self):
        """Retrieves config values from ConfigParser and sets local variables."""

        vars = super().get_all()

        for k, v in vars:
            self.variables[k] = v

    def on_default(self):
        """Resets config file and local variables to default values."""

        super().set_all(SkyDefaults.DEFAULT)
        self._write()
        self._get_all()

    def set_values(self, values):
        """
        Sets local and written variables for the given values.
        
        Args:
            values: Dictionary of name-value pairs.
        """

        for k, v in values.items():
            self.variables[k] = v
            
        super().set_all(self.variables)
        self._write()
