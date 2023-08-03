from res.constants import ColorConstants, StyleConstants
from widgets.tab import SkyTabbedPannel
from widgets.terminal import SkyTerminal
from widgets.layouts import SkyHorizontalLayout, SkyVerticalLayout
from interfaces.testing.test import IperfForm
from interfaces.testing.connection import SSHConnectionForm

from kivy.properties import ObjectProperty 
from kivy.lang.builder import Builder

Builder.load_string("""
#: import ColorConstants res.constants.ColorConstants
#: import StyleConstants res.constants.StyleConstants
                    
<SkyTestingTab>:
    text: 'Testing'
    SkyHorizontalLayout:
        canvas.before:
            Color:
                rgba: ColorConstants.tab_bg_color
            Rectangle:
                size: self.size
                pos: self.pos
        padding: StyleConstants.large_padding, StyleConstants.large_padding
        spacing: StyleConstants.huge_padding
        SkyTerminal:
            size_hint: 0.6, 1
        SkyVerticalLayout:
            size_hint: 0.4, 1
            spacing: StyleConstants.huge_padding
            SkyVerticalLayout:
                spacing: StyleConstants.large_padding
                padding: StyleConstants.large_padding, StyleConstants.large_padding
                canvas.before:
                    Color:
                        rgba: ColorConstants.form_bg_color
                    Rectangle:
                        size: self.size
                        pos:self.pos
                IperfTestForm:
                    button_label: 'Run Drone Test'
                    connection: root.drone_connection
                IperfTestForm:
                    button_label: "Run Car Test"
                    connection: root.car_connection
            SSHConnectionForm:
                car_connection: root.car_connection
                drone_connection: root.drone_connection

""")

class SkyTestingTab(SkyTabbedPannel):
    drone_connection = ObjectProperty()
    car_connection = ObjectProperty()
