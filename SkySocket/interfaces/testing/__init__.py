"""A tab for testing connection bandwidth and running SSH commands."""

from kivy.lang.builder import Builder
from kivy.properties import ObjectProperty 

from interfaces.testing.connection import SSHConnectionForm
from interfaces.testing.test import IperfForm
from widgets.tab import SkyTabbedPanel
from widgets.terminal import SkyTerminal

Builder.load_string("""
#: import ColorConstants res.constants.ColorConstants
#: import StyleConstants res.constants.StyleConstants
                    
<SkyTestingTab>:
    text: "Testing"
    SkyHorizontalLayout:
        canvas.before:
            Color:
                rgba: ColorConstants.tab_bg_color
            Rectangle:
                pos: self.pos
                size: self.size
        padding: StyleConstants.large_padding, StyleConstants.large_padding
        spacing: StyleConstants.huge_padding
        SkyTerminal:
            size_hint: 0.6, 1
        SkyVerticalLayout:
            size_hint: 0.4, 1
            spacing: StyleConstants.huge_padding
            SkyVerticalLayout:
                padding: StyleConstants.large_padding, StyleConstants.large_padding
                spacing: StyleConstants.large_padding
                canvas.before:
                    Color:
                        rgba: ColorConstants.form_bg_color
                    Rectangle:
                        pos:self.pos
                        size: self.size
                IperfTestForm:
                    connection: root.drone_connection
                    form_name: "Drone"
                IperfTestForm:
                    connection: root.car_connection
                    form_name: "Car"
            SSHConnectionForm:
                car_connection: root.car_connection
                drone_connection: root.drone_connection
""")


class SkyTestingTab(SkyTabbedPanel):
    """
    The layout for the testing tab.
    
    Attributes:
        car_connection: Car's SSHConnection object.
        drone_connection: Drone's SSHConnection object."""

    car_connection = ObjectProperty()
    drone_connection = ObjectProperty()
