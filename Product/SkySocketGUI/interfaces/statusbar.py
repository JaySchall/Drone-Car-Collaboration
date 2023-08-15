"""StatusBar definition and supporting widgets."""

from kivy.lang.builder import Builder
from kivy.properties import ColorProperty
from kivy.properties import ObjectProperty
from kivy.properties import StringProperty
from kivy.uix.label import Label

from res.constants import ColorConstants
from widgets.layouts import SkyHorizontalLayout

SESSION_NAME = "python"

Builder.load_string("""
#: import calculate_size res.constants.calculate_size
#: import ColorConstants res.constants.ColorConstants
#: import SizeConstants res.constants.SizeConstants
#: import StyleConstants res.constants.StyleConstants
                    
<Spacer>:
    size: SizeConstants.spacer_size, SizeConstants.spacer_size
    size_hint: None, None

<SkyLabel>:
    color: ColorConstants.black    
    font_size: sp(calculate_size(self.size[1]))
    size_hint: None, 1
    valign: "middle"
    width: self.texture_size[0]

<SkyStatusBar>:
    size_hint: 1, 0.05
    spacing: StyleConstants.def_padding    
    canvas.before:
        Color:
            rgba: self.bg_color
        Rectangle:
            pos: self.pos  
            size: self.size
    Spacer:
    SkyLabel:
        text: "Drone Status:"
    SkyLabel:
        text: root.drone_status
    Label:
        size_hint: 0.1, None
    SkyLabel:
        text: "Battery:"
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
    """A dummy label to take up space between other labels."""

    pass


class SkyLabel(Label):
    """A preconfigured label for use in the SkyStatusBar."""

    pass


class SkyStatusBar(SkyHorizontalLayout):
    """
    A bar at the top of the GUI that provides connection and hardware status.

    Attributes:
        bg_color: Holds the color the whole status bar is.
        car_connection: The car SSHConnection object.
        drone_connection: The drone SSHConnection object.
        battery_life: A string holding the current battery life or - if none.
        car_status: A string holding the current status of the car.
        drone_status: A string holding the current status fo the drone.
        start_status: The simulation tab.
    """

    bg_color = ColorProperty(ColorConstants.red)
    car_connection = ObjectProperty()
    drone_connection = ObjectProperty()
    
    battery_life = StringProperty("-")
    car_status = StringProperty("NOT CONNECTED")
    drone_status = StringProperty("NOT CONNECTED")
    start_status = ObjectProperty()

    def on_kv_post(self, base_widget):
        """Initial bindings post kivy startup."""

        self.drone_connection.bind(connected=self._drone_update)
        self.drone_connection.bind(online=self._drone_update)
        self.car_connection.bind(connected=self._car_update)
        self.car_connection.bind(online=self._car_update)
        self.bind(car_status=self._update)
        self.bind(drone_status=self._update)
        
        return super().on_kv_post(base_widget)

    def _drone_update(self, *args):
        """
        Reassesses the drone status if an IP comes online and updates StatusBar.
        
        dt and args are both passed via the automatic method call and aren't
        used for this method.
        """

        temp_drone = StringProperty()
        if self.drone_connection.online:
            if self.drone_connection.is_running(SESSION_NAME):
                temp_drone = "ACTIVE"
            elif self.drone_connection.connected:
                temp_drone = "CONNECTED"
            else:
                temp_drone = "ONLINE"
        else:
            temp_drone = "NOT CONNECTED"
        self.drone_status = temp_drone

    def _car_update(self, *args):
        """
        Reassesses the car status if an IP comes online and updates StatusBar.
        
        dt and args are both passed via the automatic method call and aren't
        used for this method.
        """

        if self.car_connection.online:
            if self.car_connection.is_running(SESSION_NAME):
                temp_car = "ACTIVE"
            elif self.car_connection.connected:
                temp_car = "CONNECTED"
            else:
                temp_car = "ONLINE"
        else:
            temp_car = "NOT CONNECTED"
        self.car_status = temp_car

    def _update(self, *args):
        """Checks to see if bg_color needs to be updated."""

        if self.car_status == "NOT CONNECTED" or self.drone_status == "NOT CONNECTED":
            self.start_status._disable_button()
            self.bg_color = ColorConstants.red
        elif self.car_status == "ONLINE" or self.drone_status == "ONLINE":
            self.bg_color = ColorConstants.orange
        else:
            self.start_status._do_start_button()
            self.bg_color = ColorConstants.green