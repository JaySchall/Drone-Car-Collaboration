from kivy.properties import StringProperty
from kivy.event import EventDispatcher
from kivy.metrics import dp
from kivy.metrics import sp

from res.config import SkyConfig

class ColorConstants:
    white = [1, 1, 1, 1]
    black = [0, 0, 0, 1]
    red = [1, 0.4, 0.4, 1]
    blue = [0, 0, 1, 1]
    green = [0, 1, 0.5, 1]
    orange = [1, 0.75, 0, 1]
    
    tab_bg_color = [0.85, 0.85, 0.85, 1]
    tab_bg_clicked_color = [0.74, 0.74, 0.74, 1]
    form_fg_color = [0.93,0.93,0.93,1]
    form_bg_color = [0.8, 0.8, 0.8, 1]

class StyleConstants:
    def_padding = dp(5)
    large_padding = dp(10)
    huge_padding = dp(15)
    settings_width = dp(150)
    video_width = dp(640)
    video_height = dp(360)
    cursor_size = sp(30)
    center_pos = {'center_x': 0.5, 'center_y': 0.5}

class SkyVariables(SkyConfig, EventDispatcher):
    drone_username = StringProperty()
    drone_ip = StringProperty()
    drone_password = StringProperty()
    car_username = StringProperty()
    car_ip = StringProperty()
    car_passowrd = StringProperty()
    video_source = StringProperty()
    offloading_percent = StringProperty()
    log_directory = StringProperty()
    results_directory = StringProperty()
    drone_file_directory = StringProperty()
    drone_file_name = StringProperty()
    car_file_directory = StringProperty()
    car_file_name = StringProperty()
    
    def __init__(self):
        super().__init__()
        self._get_all()
    
    def _get_all(self):
        self.drone_username = self.get_ssh('drone_username')
        self.drone_ip = self.get_ssh('drone_ip')
        self.drone_password = self.get_ssh('drone_password')
        self.car_username = self.get_ssh('car_username')
        self.car_ip = self.get_ssh('car_ip')
        self.car_passowrd = self.get_ssh('car_password')
        self.video_source = self.get_testing('video_source')
        self.offloading_percent = self.get_testing('offloading_percent')
        self.log_directory = self.get_path('log_directory')
        self.results_directory = self.get_path('results_directory')
        self.drone_file_directory = self.get_path('drone_file_directory')
        self.drone_file_name = self.get_path('drone_file_name')
        self.car_file_directory = self.get_path('car_file_directory')
        self.car_file_name = self.get_path('car_file_name')

    def on_apply(self):
        self.set_ssh('drone_username', self.drone_username)
        self.set_ssh('drone_ip', self.drone_ip)
        self.set_ssh('drone_password', self.drone_password)
        self.set_ssh('car_username', self.car_username)
        self.set_ssh('car_ip', self.car_ip)
        self.set_ssh('car_passowrd', self.car_passowrd)
        self.set_testing('video_source', self.video_source)
        self.set_testing('offloading_percent', self.offloading_percent)
        self.set_path('log_directory', self.log_directory)
        self.set_path('results_directory', self.results_directory)
        self.set_path('drone_file_name', self.drone_file_name)
        self.set_path('car_file_directory', self.car_file_directory)
        self.set_path('car_file_name', self.car_file_name)
        self._write()