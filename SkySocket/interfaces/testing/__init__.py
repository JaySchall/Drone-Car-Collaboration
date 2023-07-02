from res.constants import ColorConstants, StyleConstants
from widgets.tab import SkyTabbedPannel
from widgets.terminal import SkyTerminal
from widgets.layouts import SkyHorizontalLayout, SkyVerticalLayout
from interfaces.testing.test import IperfForm
from interfaces.testing.connection import SSHConnectionForm

from kivy.properties import ObjectProperty 
from kivy.lang.builder import Builder

Builder.load_string("""
<SkyTestingTab>:
    text: 'Testing'
    SkyHorizontalLayout:
        canvas.before:
            Color:
                rgba: root.tab_bg
            Rectangle:
                size: self.size
                pos: self.pos
        SkyTerminal:
        SkyVerticalLayout:
            spacing: root.huge_padding
            SkyVerticalLayout:
                spacing: root.def_padding
                padding: root.large_padding, root.large_padding
                canvas.before:
                    Color:
                        rgba: root.form_bg
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

    tab_bg = ColorConstants.tab_bg_color
    form_bg = ColorConstants.form_bg_color
    huge_padding = StyleConstants.huge_padding
    large_padding = StyleConstants.large_padding
    def_padding = StyleConstants.def_padding