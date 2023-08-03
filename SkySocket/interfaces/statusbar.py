from kivy.properties import StringProperty, ObjectProperty, ColorProperty
from kivy.lang.builder import Builder
from kivy.uix.label import Label
from kivy.metrics import dp
from kivy.clock import Clock

from widgets.layouts import SkyHorizontalLayout
from res.constants import StyleConstants, ColorConstants

Builder.load_string("""
<Spacer>:
    size_hint: None, None
    size: dp(20), dp(20)

<SkyLabel>:
    size_hint: None, 1
    color: self.black    
    width: self.texture_size[0]
    font_size: sp((self.size[1]/5) + 12)
    valign: "middle"

<SkyStatusBar>:
    size_hint: 1, 0.05
    spacing: self.spacing_size    
    canvas.before:
        Color:
            rgba: self.bg_color
        Rectangle:
            size: self.size
            pos: self.pos  
    Spacer:
    SkyLabel:
        text: 'Drone Status:'
    SkyLabel:
        text: root.drone_status
    Label:
        size_hint: 0.1, None
    SkyLabel:
        text: 'Battery:'
    SkyLabel:
        text: root.battery_life
    Label:
        size_hint: 0.9, None
    SkyLabel:
        text: "Car Status:"
    SkyLabel:
        text: root.car_status
    Spacer:
""")

class Spacer(Label):
    pass

class SkyLabel(Label):
    black = ColorConstants.black

class SkyStatusBar(SkyHorizontalLayout):
    spacing_size = StyleConstants.def_padding
    bg_color = ColorProperty()
    drone_connection = ObjectProperty()
    car_connection = ObjectProperty()
    start_status = ObjectProperty()

    car_status = StringProperty("NOT CONNECTED")
    drone_status = StringProperty("NOT CONNECTED")
    battery_life = StringProperty("-")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.bg_color = ColorConstants.red        
        Clock.schedule_once(self._init, 1)

    def _init(self, *args):
        self.drone_connection.bind(connected=self._drone_update)
        self.drone_connection.bind(online=self._drone_update)
        self.car_connection.bind(connected=self._car_update)
        self.car_connection.bind(online=self._car_update)

    def _drone_update(self, dt, *args):
        temp_drone = StringProperty()
        if self.drone_connection.online:
            if self.drone_connection.is_running():
                temp_drone = "ACTIVE"
            elif self.drone_connection.connected:
                temp_drone = "CONNECTED"
            else:
                temp_drone = "ONLINE"
        else:
            temp_drone = "NOT CONNECTED"
        self.drone_status = temp_drone
        self._update()

    def _car_update(self, dt, *args):
        if self.car_connection.online:
            if self.car_connection.is_running():
                temp_car = "ACTIVE"
            elif self.car_connection.connected:
                temp_car = "CONNECTED"
            else:
                temp_car = "ONLINE"
        else:
            temp_car = "NOT CONNECTED"
        self.car_status = temp_car
        self._update()

    def _update(self):
        if self.car_status == "NOT CONNECTED" or self.drone_status == "NOT CONNECTED":
            self.start_status._dissable_button()
            self.bg_color = ColorConstants.red
        elif self.car_status == "ONLINE" or self.drone_status == "ONLINE":
            self.bg_color = ColorConstants.orange
        else:
            self.start_status._do_start_button()
            self.bg_color = ColorConstants.green