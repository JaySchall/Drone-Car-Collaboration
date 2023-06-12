from kivy import Config

Config.set('graphics', 'minimum_width', '1050')
Config.set('graphics', 'minimum_height', '650')


import kivy
from kivy.uix.boxlayout import BoxLayout
from kivy.app import App
from kivy.properties import ObjectProperty, StringProperty
from kivy.metrics import dp

from kivy.core.window import Window
from kivy.modules import inspector

from widgets.tab import SkyTabbedPannel

class MainWindow(BoxLayout):
    pass


class SkySocketApp(App):

    def build(self):
        return MainWindow()
    
    def on_start(self):
        inspector.create_inspector(Window, self)
    
if __name__ == '__main__':
    SkySocketApp().run()