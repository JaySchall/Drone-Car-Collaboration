from kivy.properties import StringProperty
from kivy.event import EventDispatcher
from kivy.metrics import dp
from kivy.metrics import sp

from res.config import SkyConfig
from res.config import SkyDefaults

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

    button_norm = [0.62, 0.62, 0.62, 62]
    button_down = [0.5, 0.5, 0.5, 1]
    button_dissabled = [0.7, 0.7, 0.7, 1]
    button_down_dissabled = [0.6, 0.6, 0.6, 1]

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
    variables = {}
    __getattr__ = variables.get
    __setattr__ = variables.__setitem__
    
    def __init__(self):
        super().__init__()
        self._get_all()
    
    def _get_all(self):
        vars = super().get_all()

        for k, v in vars:
            self.variables[k] = v

    def on_default(self):
        super().set_all(SkyDefaults.DEFAULT)
        self._write()

    def set_values(self, values):
        for k, v in values.items():
            self.variables[k] = v
        super().set_all(self.variables)
        self._write()