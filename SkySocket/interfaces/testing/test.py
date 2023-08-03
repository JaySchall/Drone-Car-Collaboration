from res.constants import ColorConstants
from widgets.layouts import SkyVerticalLayout
from widgets.form import SkyForm
from widgets.form.formtable import SkyFormTable
from widgets.form.fields import SkyCheckBox, SkyTextInput, SkyBrowse
from res.constants import StyleConstants

from kivy.properties import ObjectProperty, StringProperty
from kivy.metrics import dp, sp

from kivy.lang.builder import Builder

Builder.load_string("""
<IperfForm>:
    size_hint: 1, 1

<IperfTestForm>:
    canvas.before:
        Color:
            rgba: self.form_fg
        Rectangle:
            size: self.size
            pos:self.pos
    IperfForm:
        id: ipf
    AnchorLayout:
        anchor_x: "center"
        anchor_y: "center"
        size_hint: 1, 0.28
        Button:
            text: root.button_label
            background_color: root.bg
            background_normal: ''
            color: root.black
            size_hint: 0.4, 0.75
            min_size_hint_x: dp(120)
            pos_hint: {'center': 0.5, 'center': 0.5}
            font_size: sp((self.size[1]/5) + 12)
            on_release: root.iperf_test(ipf.form.get_values())
""")


class IperfTestForm(SkyVerticalLayout):
    form_fg = ColorConstants.form_fg_color
    bg = ColorConstants.tab_bg_color
    black = ColorConstants.black

    connection = ObjectProperty()
    button_label = StringProperty()

    def iperf_test(self, flags):
        server_args = ['iperf3', '-s']
        client_args = ['iperf3', '-c', '-t', '30']
        if flags['port']:
            client_args.append('-p')
            client_args.append(flags['port'])
            server_args.append('-p')
            server_args.append(flags['port'])
        if flags['verbose']:
            client_args.append('-V')
            server_args.append('-V')
        if flags['server']:
            if flags['logfile']:
                server_args.append('--logfile')
                server_args.append('iperf3_TCP_server.txt')
        else:
            if flags['logfile']:
                client_args.append('--logfile')
                client_args.append('iperf3_TCP_client.txt')
        #IMPL threaded call to run both commands. after one ends, terminate the other
        print(" ".join(server_args))
        print(" ".join(client_args))


class IperfForm(SkyFormTable):
    returned = StringProperty()
    label_width = dp(150)
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.form = SkyForm()
        self.add_field(SkyCheckBox(sid = "verbose", label = "Verbose", size_hint = (None, 1)), size_hint = (1, 1))
        self.add_field(SkyCheckBox(sid = "server", label = "Drone as server", size_hint = (None, 1)), size_hint = (1, 1))
        self.add_field(SkyTextInput(sid = "port", label = "Port", input_filter = "int", height = dp(25),
                                     size_hint = (0.66, 0), pos_hint = {"center_y": 0.5}), size_hint = (1, 1))
        self.add_field(SkyCheckBox(sid = "logfile", label = "Logfile", size_hint = (None, 1)), size_hint = (1, 1))
            