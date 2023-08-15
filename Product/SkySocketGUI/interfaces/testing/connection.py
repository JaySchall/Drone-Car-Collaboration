"""
The connection form to run commands on a remote host.

This has not been fully implemented. There may need to be more specification as
to the types of information shown on the connection form (status, last command
ran, etc.). Basic building blocks have been implemented, but no substantial
code has been written regarding user controlled SSH connections.
"""

from kivy.lang.builder import Builder
from kivy.metrics import dp
from kivy.properties import ObjectProperty

from mixins.ssh import SSHConnection
from widgets.form import SkyForm
from widgets.form.fields import SkyTextInput
from widgets.form.formtable import SkyFormTable
from widgets.layouts import SkyVerticalLayout

Builder.load_string("""          
#: import calculate_size res.constants.calculate_size
#: import ColorConstants res.constants.ColorConstants
#: import StyleConstants res.constants.StyleConstants

<ConnectForm>:
                    
<SSHConnectionForm>:
    padding: StyleConstants.large_padding, StyleConstants.large_padding
    spacing: StyleConstants.def_padding
    canvas.before:
        Color:
            rgba: ColorConstants.form_bg_color
        Rectangle:
            pos:self.pos
            size: self.size
    SkyVerticalLayout:
        padding: StyleConstants.large_padding, StyleConstants.large_padding
        spacing: StyleConstants.def_padding
        canvas.before:
            Color:
                rgba: ColorConstants.form_fg_color
            Rectangle:
                pos:self.pos
                size: self.size
        ConnectForm:
            id: conform
            size_hint: 1, 0.4
        SkyHorizontalLayout:
            size_hint: 1, 0.2
            Spinner:
                id: valuepull
                background_color: ColorConstants.tab_bg_color
                background_normal: ""
                color: ColorConstants.black
                font_size: sp(calculate_size(self.size[1]))
                pos_hint: {"center_y": 0.5}
                size_hint: 0.35, 0.6
                text: "Manual"
                values: "Manual", "Drone Default", "Car Default"
            Label:
                size_hint: 0.30, 0.6
            Button:
                background_color: ColorConstants.tab_bg_color
                background_normal: ""
                color: ColorConstants.black
                font_size: sp(calculate_size(self.size[1]))
                pos_hint: {"center_y": 0.5}
                size_hint: .35, 0.6
                text: "CONNECT"
                on_release: root.connect(conform.form.get_values(), valuepull.text)
        TextInput:
            size_hint: 1, 0.4
            text_hint: "Enter commands..."
""")


class ConnectForm(SkyFormTable):
    """FormTable for SSH connection parameters."""

    def __init__(self, **kwargs):
        """Initializes form with fields."""

        super().__init__(**kwargs)
        self.form = SkyForm()
        self.add_field(SkyTextInput(sid="user", label="User", 
                                    height=dp(25),
                                    pos_hint={"center_y": 0.5},
                                    size_hint=(0.66, None)), 
                                    size_hint=(1, 1))
        self.add_field(SkyTextInput(sid="ip", label="IP",  
                                    height=dp(25),
                                    pos_hint={"center_y": 0.5},
                                    size_hint=(0.66, None)), 
                                    size_hint=(1, 1))
        self.add_field(SkyTextInput(sid="password", label="Password", 
                                    height=dp(25),
                                    password=True,
                                    pos_hint={"center_y": 0.5},
                                    size_hint=(0.66, None)), 
                                    size_hint=(1, 1))


class SSHConnectionForm(SkyVerticalLayout):
    """
    Controls and monitors SSH connection.

    Attributes:
        def_connection: The connection to be used for communication.
        car_connection: The car SSHConnection object.
        drone_connection: The drone SSHConnection object.
    """

    def_connection = ObjectProperty()
    car_connection = ObjectProperty()
    drone_connection = ObjectProperty()

    def connect(self, values, valuepull):
        """
        Sets user defined connection parameters and establishes SSH connection.

        Args:
            values: The user input paramteres for a connection.
            valluepull: The result of the spinbox deciding if the user's inputs,
                car defaults, or drone defaults will be used to connect.
        """

        if valuepull == "Manual":
            if values["user"] == "" or values["ip"] == "" or values["password"] == "":
                print("Please give an input for all manual values")
                return
            self.def_connection = SSHConnection(values["ip"], values["user"], values["password"])
        elif valuepull == "Drone Default":
            self.def_connection = self.drone_connection
        elif valuepull == "Car Default":
            self.def_connection = self.car_connection
        
        print(self.def_connection)
