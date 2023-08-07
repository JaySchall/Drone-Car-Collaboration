"""Hardware connection status info and button layout."""

from kivy.lang.builder import Builder
from kivy.properties import StringProperty
from kivy.properties import ObjectProperty
from kivy.uix.button import Button
from kivy.uix.label import Label

from res.constants import ColorConstants
from widgets.layouts import SkyHorizontalLayout

Builder.load_string("""
#: import ColorConstants res.constants.ColorConstants

<SimulationButton>:
    background_color: ColorConstants.button_norm    
    background_down: ""       
    background_normal: ""
    font_size: sp(self.height/3.2 + 10)
    pos_hint: StyleConstants.center_pos
    size_hint: 0.8, 0.8

<StatusLabel>:
    bold: True
    color: ColorConstants.black
    halign: "center"
    size: self.texture_size
    text_size: self.size
    valign: "bottom"

<StatusResult>:
    color: ColorConstants.black
    halign: "center"
    size: self.texture_size
    text_size: self.size
    valign: "top"

<SkyConnectForm>:
    pos_hint: StyleConstants.center_pos
    canvas.before:
        Color:
            rgba: ColorConstants.form_fg_color
        Rectangle:
            pos: self.pos
            size: self.size
    SkyVerticalLayout:
        padding: dp(0), StyleConstants.huge_padding
        size_hint: 0.4, 1
        spacing: StyleConstants.huge_padding
        SimulationButton:
            id: connect
            text: "Connect"
            on_release: root.on_connect()
        SimulationButton:
            text: "Ping"
            on_release: root.on_ping()
        SimulationButton:
            text: "Restart"
            on_release: root.on_restart()
    SkyVerticalLayout:
        size_hint: 0.6, 1
        StatusLabel:
            font_size: (connect.font_size/3)*2
            text: "Connected:"
        StatusResult:
            id: connectlabel
            font_size: (connect.font_size/3)*2
            text: root.connected_label
        StatusLabel:
            font_size: (connect.font_size/3)*2
            text: "Online:"
        StatusResult:
            id: statuslabel
            font_size: (connect.font_size/3)*2
            text: root.active_label
""")


class SimulationButton(Button):
    """Buttons for connection form."""

    def on_press(self, *args):
        """Change button color when pressed."""
        super().on_press()
        self.background_color = ColorConstants.button_down
     
    def on_touch_up(self, *args):
        """Change button color when mouse releases."""

        super().on_touch_up(*args)
        self.background_color = ColorConstants.button_norm


class StatusLabel(Label):
    """Bolded label for type of connection status."""
    
    pass


class StatusResult(Label):
    """Label containing actual connection status."""

    pass


class SkyConnectForm(SkyHorizontalLayout):
    """
    Layout containing hardware status and buttons to change said status.
    
    Attributes:
        active_label: Label displaying if the believed IP is online or not.
        connected_label: Label displaying if the hardware has an active SSH
            connection or not.
        connection: The SSHConnection object for the relevant hardware.
    """

    active_label = StringProperty("FALSE")
    connected_label = StringProperty("FALSE")
    connection = ObjectProperty()

    def on_kv_post(self, base_widget):
        """Sets up automatic bindings after Kivy is parsed."""

        self.connection.bind(online=self._update_active)
        self.connection.bind(connected=self._update_connect)
        
        return super().on_kv_post(base_widget)

    def _update_connect(self, *args):
        """Called when an SSH connection is made to update connection status."""

        if self.connection.connected and self.connected_label == "FALSE":
            self.connected_label = "TRUE"
        elif not self.connection.connected and self.connected_label == "TRUE":
            self.connected_label = "FALSE"

    def _update_active(self, *args):
        """Called when an IP comes online to update online status."""

        if self.connection.online and self.active_label == "FALSE":
            self.active_label = "TRUE"
        elif not self.connection.online and self.active_label == "TRUE":
            self.active_label = "FALSE"

    def on_ping(self, *arg):
        """Pings the forms IP to see if it's online."""

        self.connection.on_ping()

    def on_connect(self, *args):
        """Attempts to establish an SSH connection with the hardware."""
        
        self.connection.reconnect()

    def on_restart(self, *args):
        """Restarts the hardware."""
        
        self.connection.cmd("sudo shutdown -r now")