"""
The logic and forms to run an iPerf3 bandwidth test.

This has not been fully implemented. The file retrieval/output, threading, and
the actual test commands has not been touched. Only user input has been created 
and tested.
"""

from kivy.lang.builder import Builder
from kivy.properties import ObjectProperty
from kivy.properties import StringProperty

from widgets.form import SkyForm
from widgets.form.fields import SkyCheckBox
from widgets.form.fields import SkyTextInput
from widgets.form.formtable import SkyFormTable
from widgets.layouts import SkyVerticalLayout

Builder.load_string("""
#: import calculate_size res.constants.calculate_size
#: import ColorConstants res.constants.ColorConstants
#: import StyleConstants res.constants.StyleConstants

<IperfForm>:
    size_hint: 1, 1

<IperfTestForm>:
    canvas.before:
        Color:
            rgba: ColorConstants.form_fg_color
        Rectangle:
            pos:self.pos
            size: self.size
    AnchorLayout:
        anchor_x: "center"
        anchor_y: "center"
        size_hint: 1, 0.28
        Button:
            background_color: ColorConstants.tab_bg_color
            background_normal: ""
            color: ColorConstants.black
            font_size: sp(calculate_size(self.size[1]))
            min_size_hint_x: SizeConstants.min_start
            pos_hint: StyleConstants.center_pos
            size_hint: 0.4, 0.75
            text: f"Run {root.form_name} Test"
            on_release: root.iperf_test(ipf.form.get_values())
""")


class IperfTestForm(SkyVerticalLayout):
    """
    The form for layout and logic for running an iPerf3 test.
    
    Attributes:
        connection: The SSHConnection object relevant to this form.
        form_name: The device the form is for.
    """

    connection = ObjectProperty()
    form_name = StringProperty()

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.bind(form_name=self.init_form)

    def init_form(self, *args):
        """Creates form to be rendered."""

        self.add_widget(IperfForm(self.form_name), 1)

    def iperf_test(self, flags):
        """
        Called on user test request. Creates commands and starts threads.
        
        Args:
            flags: A dictionary of flag-value pairs.
        """

        # Default base commands for the server and client
        server_args = ["iperf3", "-s"]
        client_args = ["iperf3", "-c", "-t", "30"]

        if flags["port"]:
            client_args.append("-p")
            client_args.append(flags["port"])
            server_args.append("-p")
            server_args.append(flags["port"])
        if flags["verbose"]:
            client_args.append("-V")
            server_args.append("-V")
        if flags["server"]:
            if flags["logfile"]:
                server_args.append("--logfile")
                server_args.append("iperf3_TCP_server.txt")
        else:
            if flags["logfile"]:
                client_args.append("--logfile")
                client_args.append("iperf3_TCP_client.txt")

        # TODO: threaded call to run both commands. After one ends, terminate 
        # the other. One thread for computer side (server or client depending
        # on the "as server", input).
        print(" ".join(server_args))
        print(" ".join(client_args))


class IperfForm(SkyFormTable):
    """
    The FormTable for the user to set the test flags.
    """

    def __init__(self, form_name, **kwargs):
        """Creates and configures table."""

        super().__init__(**kwargs)
        self.form = SkyForm()
        self.add_field(SkyCheckBox(sid="verbose", label="Verbose",
                                   size_hint=(None, 1)),
                                   size_hint=(1, 1))
        self.add_field(SkyCheckBox(sid="server", label=f"{form_name} as server",
                                   size_hint=(None, 1)),
                                   size_hint=(1, 1))
        self.add_field(SkyTextInput(sid="port", label="Port",
                                    input_filter="int",
                                    pos_hint={"center_y": 0.5},
                                    size_hint=(0.66, 0)),
                                    size_hint=(1, 1))
        self.add_field(SkyCheckBox(sid="logfile", label="Logfile", 
                                   size_hint=(None, 1)), 
                                   size_hint=(1, 1))
            