from res.constants import ColorConstants, StyleConstants
from widgets.tab import SkyTabbedPannel
from widgets.video import SkyVideoPlayer
from widgets.terminal import SkyTerminal
from widgets.layouts import SkyHorizontalLayout, SkyVerticalLayout
from interfaces.simulation.settings import SimulationSettingsForm
from interfaces.simulation.connect import SkyConnectForm

from kivy.properties import ObjectProperty, StringProperty
from kivy.metrics import dp
from kivy.lang.builder import Builder

Builder.load_string("""
<SkySimulationTab>:
    text: 'Simulation'
    SkyHorizontalLayout:
        SkyVerticalLayout:
            padding: root.def_padding, root.def_padding
            spacing: root.def_padding
            size_hint_min_x: root.video_width + root.large_padding
            SkyVideoPlayer:
                video_width: root.video_width
                video_height: root.video_height
            Label:
                id: video_ip
                color: root.black
                size_hint: None, None
                size: self.texture_size
                halign: 'left'
                text: root.link
            SkyTerminal:
        SkyVerticalLayout:
            padding: root.def_padding, root.def_padding
            spacing: root.def_padding
            size_hint_min_x: dp(350)
            size_hint_max_x: dp(450)
            SkyVerticalLayout:
                canvas.before:
                    Color:
                        rgba: root.form_bg_color
                    Rectangle:
                        size: self.size
                        pos: self.pos 
                SkyConnectForm:
                    connection: root.drone_connection
                SkyConnectForm:
                    connection: root.car_connection
                SimulationSettingsForm:
                    id: simulation_settings            
            Button:
                text: 'START'
                padding_y: dp(100)
                size_hint: 0.8, 0.2
                pos_hint: root.center_pos
                on_release: root.on_start()
                background_color: root.green
                font_size: self.height/2
                bold: True
""")

class SkySimulationTab(SkyTabbedPannel):
    
    drone_connection = ObjectProperty()
    car_connection = ObjectProperty()
    link = StringProperty()
    
    black = ColorConstants.black
    def_padding = StyleConstants.def_padding
    large_padding = StyleConstants.large_padding
    video_width = StyleConstants.video_width
    video_height = StyleConstants.video_height
    green = ColorConstants.green
    center_pos = StyleConstants.center_pos
    form_bg_color = ColorConstants.form_bg_color
    
    def on_start(self):
        print('start')

