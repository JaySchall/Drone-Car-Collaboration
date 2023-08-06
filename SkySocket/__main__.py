"""
A GUI for controlling a Raspberry Pi based drone-car collaboration experiment.

This program allows for control of a programmable drone and car for research
purposes. This was designed with the intention of use for communication,
computation, and detection studies. It allows for users to run scripts, 
communicate with the hardware, and run bandwidth testing.
"""

from kivy import Config

# Defines minimum size for window
MINIMUM_SIZE = (1120, 672)
Config.set("graphics", "minimum_width", str(MINIMUM_SIZE[0]))
Config.set("graphics", "minimum_height", str(MINIMUM_SIZE[1]))

import kivy
from kivy.app import App
from kivy.core.window import Window
from kivy.metrics import dp
from kivy.modules import inspector
from kivy.uix.boxlayout import BoxLayout

from mixins.ssh import SSHConnection
from res.config import SkyVariables


class MainWindow(BoxLayout):
    """
    The layout for the entire window.

    Attributes:
        settings: A SkyVariables that contains the sessions settings access.
        drone: An SSHConnection with the drone's data and connection module.
        car: An SSHConnection with the car's data and connection module.
    """

    settings = SkyVariables()
    car = SSHConnection(settings.car_ip,
                        settings.car_username,
                        settings.car_password)
    drone = SSHConnection(settings.drone_ip,
                            settings.drone_username,
                            settings.drone_password)
    
    def __init__(self, **kwargs):
        """
        Creates the layout for the programs entire window.
        """

        super().__init__(**kwargs)

        # Creates 20 pixels of space before the tabs and removes the default 2
        # pixels of space between a tab and it's layout object.
        self.ids.tab_panel._tab_layout.padding = [dp(20), 0, 0, dp(-2)]
        # Adds 15 pixels of space between tabs.
        self.ids.tab_panel._tab_strip.spacing = (dp(15), 0)


class SkySocketApp(App):
    """
    The central logic that initiates Kivy and holds app wide logic.

    Attributes:
        last_size: A numeric tuple with the dimensions of the window from the 
            last frame it was changed.
        main_window: MainWindow holding top level box layout of the window.
    """
    
    last_size = MINIMUM_SIZE
    
    def build(self):
        """
        See build documentation from Kivy for more information.

        Sets up bindings for custom automation for window actions, builds the
        window and returns it.
        """

        Window.bind(on_request_close=self.on_request_close)
        Window.bind(on_pre_resize = self._bind_size)
        self.main_window = MainWindow()
        return self.main_window
    
    def on_start(self):
        """See on_start documentation from Kivy for more information."""

        # Only for dev use. Press control + E to open inspector widget
        inspector.create_inspector(Window, self)

    def on_request_close(self, *args):
        """
        Process to close SSH connections on program exit.
        """
        
        if self.main_window.drone.is_connected():
            self.main_window.drone.disconnect()
        else:
            print("drone is not connected")
        if self.main_window.car.is_connected():
            self.main_window.car.disconnect()
        else:
            print("car is not connected")

        self.stop()
        Window.close()

    def _bind_size(self, *args):
        """
        Enforces a consistent aspect ratio when resizing the window.

        To prevent unexpected results with dynamic widgets, this method enforces
        a 5 by 3 aspect ratio to ensure the user interface looks correct, all
        aspects are visually pleasing, and maintains usability.

        Args:
            *args: contains the window widget, the new width, and the new height
                in that order.
        """

        width = args[1]
        height = args[2]

        if (width, height) == self.last_size:
            return
        
        if width < MINIMUM_SIZE[0] or height < MINIMUM_SIZE[1]:
            Window._size = MINIMUM_SIZE
            return

        # Determines which dimension to prioritize in resizing the window
        if abs(width - self.last_size[0])/5 > abs(height - self.last_size[1])/3:
            Window._size = width, 3 * (width/5)
        else:
            Window._size = 5 * (height/3), height

        self.last_size = (width, height)  
        
    
if __name__ == "__main__":
    SkySocketApp().run()
