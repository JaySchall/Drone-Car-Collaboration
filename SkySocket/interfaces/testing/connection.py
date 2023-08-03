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
        padding: root.large_padding, root.large_padding
        spacing: root.def_padding
        canvas.before:
            Color:
                rgba: root.form_fg
            Rectangle:
                size: self.size
                pos:self.pos
        ConnectForm:
            id: conform
            size_hint: 1, 0.4
        SkyHorizontalLayout:
            size_hint: 1, 0.2
            Spinner:
                id: valuepull
                size_hint: 0.35, 0.6
                background_normal: ''
                background_color: root.bg
                color: root.black
                pos_hint: {'center_y': 0.5}
                font_size: sp((self.size[1]/5) + 12)
                text: 'Manual'
                values: 'Manual', 'Drone Default', 'Car Default'
            Label:
                size_hint: 0.30, 0.6
            Button:
                text: 'CONNECT'
                background_normal: ''
                background_color: root.bg
                color: root.black
                size_hint: .35, 0.6
                pos_hint: {'center_y': 0.5}
                font_size: sp((self.size[1]/5) + 12)
                on_release: root.connect(conform.form.get_values(), valuepull.text)
        TextInput:
            text_hint: 'Enter commands...'
            size_hint: 1, 0.4
""")

class ConnectForm(SkyFormTable):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.form = SkyForm()
        self.add_field(SkyTextInput(sid = "user", label = "User", size_hint = (0.66, None), height = dp(25),
                                    pos_hint = {"center_y": 0.5}), size_hint = (1, 1))
        self.add_field(SkyTextInput(sid = "ip", label = "IP", size_hint = (0.66, None), height = dp(25),
                                    pos_hint = {"center_y": 0.5}), size_hint = (1, 1))
        self.add_field(SkyTextInput(sid = "password", label = "Password", password = True,  height = dp(25),
                                    size_hint = (0.66, None), pos_hint = {"center_y": 0.5}), size_hint = (1, 1))

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
        

