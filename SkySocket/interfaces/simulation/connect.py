from widgets.layouts import SkyHorizontalLayout, SkyVerticalLayout
from res.constants import ColorConstants, StyleConstants

from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.lang.builder import Builder
from kivy.properties import StringProperty, ObjectProperty
from kivy.clock import Clock


Builder.load_string("""
#: import SkyVerticalLayout widgets.layouts.SkyVerticalLayout
<SimulationButton>:
    size_hint: 0.8, 0.8
    pos_hint: self.center_pos

<StatusLabel>:
    color: self.black
    halign: 'center'
    valign: 'bottom'

<StatusResult>:
    color: self.black
    halign: 'center'
    valign: 'top'

<SkyConnectForm>:
    canvas.before:
        Color:
            rgba: self.form_fg_color
        Rectangle:
            size: self.size
            pos: self.pos  
    pos_hint: self.center_pos
    SkyVerticalLayout:
        size_hint: 0.4, 1
        padding: dp(0), root.huge_padding
        spacing: root.huge_padding
        SimulationButton:
            text: 'Connect'
            on_release: root.on_connect()
        SimulationButton:
            text: 'Ping'
            on_release: root.on_ping()
        SimulationButton:
            text: 'Restart'
            on_release: root.on_restart()
    SkyVerticalLayout:
        size_hint: 0.6, 1
        StatusLabel:
            text: 'Connected'
        StatusResult:
            id: connectlabel
            text: root.connected_label
        StatusLabel:
            text: 'Online'
        StatusResult:
            id: statuslabel
            text: root.active_label
""")


class SimulationButton(Button):
    center_pos = StyleConstants.center_pos


class StatusLabel(Label):
    black = ColorConstants.black


class StatusResult(Label):
    black = ColorConstants.black


class SkyConnectForm(SkyHorizontalLayout):
    connected_label = StringProperty("FALSE")
    active_label = StringProperty("FALSE")
    connection = ObjectProperty()

    huge_padding = StyleConstants.huge_padding
    form_fg_color = ColorConstants.form_fg_color
    center_pos = StyleConstants.center_pos

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_once(self._init, 1)

    def _init(self, *args):
        self.connection.bind(online=self._update_active)
        self.connection.bind(connected=self._update_connect)

    def _update_connect(self, *args):
        if self.connection.connected and self.connected_label == "FALSE":
            self.connected_label = "TRUE"
        elif not self.connection.connected and self.connected_label == "TRUE":
            self.connected_label = "FALSE"

    def _update_active(self, *args):
        if self.connection.online and self.active_label == "FALSE":
            self.active_label = "TRUE"
        elif not self.connection.online and self.active_label == "TRUE":
            self.active_label = "FALSE"

    def on_ping(self):
        print("pinging")
        self.connection.on_ping()

    def on_connect(self):
        print("connecting")
        self.connection.reconnect()

    def on_restart(self):
        print('restarting')
        self.connection.cmd("sudo shutdown -r now")