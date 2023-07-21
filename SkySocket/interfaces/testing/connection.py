from res.constants import ColorConstants, StyleConstants
from widgets.layouts import SkyVerticalLayout
from widgets.form import SkyForm
from widgets.form.formtable import SkyFormTable
from widgets.form.fields import SkyTextInput
from mixins.ssh import sshConnection

from kivy.properties import ObjectProperty
from kivy.metrics import dp

from kivy.lang.builder import Builder

Builder.load_string("""
<ConnectForm>:
<SSHConnectionForm>:
    spacing: root.def_padding
    padding: root.large_padding, root.large_padding
    canvas.before:
        Color:
            rgba: self.form_bg
        Rectangle:
            size: self.size
            pos:self.pos
    SkyVerticalLayout:
        canvas.before:
            Color:
                rgba: root.form_fg
            Rectangle:
                size: self.size
                pos:self.pos
        ConnectForm:
            id: conform
        SkyHorizontalLayout:
            size_hint: None, None
            Spinner:
                id: valuepull
                size_hint: None, None
                size: dp(120), dp(30)
                background_normal: ''
                background_color: root.bg
                color: root.black
                pose_hint: {'x': 0}
                text: 'Manual'
                values: 'Manual', 'Drone Default', 'Car Default'
            Button:
                text: 'CONNECT'
                background_normal: ''
                background_color: root.bg
                color: root.black
                size_hint: None, None
                pos_hint: {'right': 0}
                size: dp(120), dp(30)
                on_release: root.connect(conform.form.get_values(), valuepull.text)
        TextInput:
            text_hint: 'Enter commands...'
""")

class ConnectForm(SkyFormTable):
    label_width = dp(150)
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.form = SkyForm()
        self.add_field(SkyTextInput(sid = "user", label = "User", width = self.label_width))
        self.add_field(SkyTextInput(sid = "ip", label = "IP", width = self.label_width))
        self.add_field(SkyTextInput(sid = "password", label = "Password", password = True, width = self.label_width))

class SSHConnectionForm(SkyVerticalLayout):
    form_fg = ColorConstants.form_fg_color
    form_bg = ColorConstants.form_bg_color
    bg = ColorConstants.tab_bg_color
    black = ColorConstants.black

    def_connection = ObjectProperty()
    car_connection = ObjectProperty()
    drone_connection = ObjectProperty()

    def_padding = StyleConstants.def_padding
    large_padding = StyleConstants.large_padding

    def connect(self, values, valuepull):
        if valuepull == 'Manual':
            if values['user'] == "" or values['ip'] == "" or values['password'] == "":
                print("Please give an input for all manual values")
                return
            self.def_connection = sshConnection(values['ip'], values['user'], values['password'])
        elif valuepull == 'Drone Default':
            self.def_connection = self.drone_connection
        elif valuepull == 'Car Default':
            self.def_connection = self.car_connection
        
        print(self.def_connection)
        

