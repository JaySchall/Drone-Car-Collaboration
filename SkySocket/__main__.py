from kivy import Config

Config.set('graphics', 'minimum_width', '1120')
Config.set('graphics', 'minimum_height', '672')


import kivy
from kivy.uix.boxlayout import BoxLayout
from kivy.app import App
from kivy.metrics import dp

from kivy.core.window import Window
from kivy.modules import inspector

from widgets.tab import SkyTabbedPannel
from interfaces.simulation import SkySimulationTab
from interfaces.testing import SkyTestingTab
from interfaces.settings import SkySettingsTab
from interfaces.statusbar import SkyStatusBar
from res.constants import SkyVariables
from mixins.ssh.car import carConnection
from mixins.ssh.drone import droneConnection

MINIMUM_SIZE = (1120, 672)

class MainWindow(BoxLayout):
    settings = SkyVariables()
    drone = droneConnection(settings.drone_ip,
                            settings.drone_username,
                            settings.drone_password)
    car = carConnection(settings.car_ip,
                        settings.car_username,
                        settings.car_passowrd)
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.ids["tabpanel"]._tab_layout.padding = [dp(20), 0, 0, dp(-2)]
        self.ids["tabpanel"]._tab_strip.spacing = (dp(15), 0)


class SkySocketApp(App):
    last_size = MINIMUM_SIZE
    def build(self):
        Window.bind(on_request_close=self.on_request_close)
        Window.bind(on_pre_resize = self._bind_size)
        self.main_window = MainWindow()
        return self.main_window
    
    def on_start(self):
        inspector.create_inspector(Window, self)

    def on_request_close(self, *args):
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
        width = args[1]
        height = args[2]

        if (width, height) == self.last_size:
            return
        
        if width < MINIMUM_SIZE[0] or height < MINIMUM_SIZE[1]:
            Window._size = MINIMUM_SIZE

        if abs(width - self.last_size[0])/5 > abs(height - self.last_size[1])/3:
            Window._size = width, 3 * (width/5)
        else:
            Window._size = 5 * (height/3), height

        self.last_size = (width, height)
            
        
    
if __name__ == '__main__':
    SkySocketApp().run()