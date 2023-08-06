"""
Simulation tab for running and monitoring active drone-car simulations.

This file contains the logic for the base of the simulation tab as well as the
logic to start, run, and stop the actual simulation in threaded SSH commands.
"""

from threading import Thread
from time import sleep

from kivy.clock import mainthread
from kivy.lang.builder import Builder
from kivy.properties import BooleanProperty
from kivy.properties import ObjectProperty
from kivy.properties import StringProperty

from interfaces.simulation.connect import SkyConnectForm
from interfaces.simulation.settings import SimulationSettingsForm
from widgets.tab import SkyTabbedPanel
from widgets.video import SkyVideoPlayer

SESSION_NAME = "python"

Builder.load_string("""
#: import ColorConstants res.constants.ColorConstants
#: import SizeConstants res.constants.SizeConstants
#: import StyleConstants res.constants.StyleConstants
                    
<SkySimulationTab>:
    text: "Simulation"
    SkyHorizontalLayout:
        padding: StyleConstants.large_padding, StyleConstants.large_padding
        SkyVerticalLayout:
            padding: StyleConstants.def_padding, StyleConstants.def_padding
            size_hint: 0.6, 1
            size_hint_min_x: SizeConstants.video_width + StyleConstants.large_padding
            spacing: StyleConstants.def_padding
            SkyVideoPlayer:
                source: root.video_source
                video_height: SizeConstants.video_height
                video_width: SizeConstants.video_width
            Label:
                id: video_ip
                color: ColorConstants.black
                halign: "left"
                size: self.texture_size
                size_hint: None, None
                text: root.video_source
            SkyTerminal:
        SkyVerticalLayout:
            padding: StyleConstants.huge_padding, StyleConstants.huge_padding
            size_hint: 0.4, 1
            size_hint_max_x: SizeConstants.connect_max_size
            size_hint_min_x: SizeConstants.connect_min_size
            spacing: StyleConstants.huge_padding
            SkyVerticalLayout:
                padding: StyleConstants.huge_padding, StyleConstants.huge_padding
                spacing: StyleConstants.huge_padding
                canvas.before:
                    Color:
                        rgba: ColorConstants.form_bg_color
                    Rectangle:
                        pos: self.pos 
                        size: self.size
                SkyConnectForm:
                    connection: root.drone_connection
                SkyConnectForm:
                    connection: root.car_connection
                SimulationSettingsForm:
                    id: simulation_settings            
            Button:
                id: startbutton
                background_color: ColorConstants.green
                bold: True
                disabled: True
                font_size: self.height/2
                padding: 0, StyleConstants.start_padding
                pos_hint: StyleConstants.center_pos
                size_hint: 0.8, 0.2
                text: "START"
                on_release: root.on_start(simulation_settings.form.get_values())
""")


class SkySimulationTab(SkyTabbedPanel):
    """
    Layout and functionality of the Simulation tab, connections, and simulation.
    
    Attributes:
        car_connection: Car SSHConnection object.
        drone_connection: Drone SSHConnection object.
        settings: The SkyVariables object with user settings.
        should_run: A boolean representing if the user has requested the
            simulation to be running or not.
        start_thread: A thread to for the start button methods to run.
        video_source: The link to the video's source stream.
    """
    
    car_connection = ObjectProperty()
    drone_connection = ObjectProperty()
    settings = ObjectProperty()
    
    should_run = BooleanProperty(False)
    start_thread = None
    video_source = StringProperty()
    
    def __init__(self, **kwargs):
        """Initialize widget and thread object."""

        super().__init__(**kwargs)
        self.running_thread = Thread(target=self._start_run)

    def on_start(self, values):
        """
        Begins or ends simulation after start button is pressed.
        
        When the program is in the process of starting or ending a simulation,
        the start button is disabled to avoid race conditions.

        Args:
            values: a dictionary of the user input values.
        """

        if not self.running_thread.is_alive():
            self._disable_button()
            self.should_run = True
            self.running_thread.run()
        else:
            self._disable_button()
            self.should_run = False

    @mainthread
    def _do_start_button(self):
        """Enables the start button to initialize a simulation."""

        temp_ref = self.ids.startbutton
        temp_ref.background_color = self.green
        temp_ref.text = "START"
        temp_ref.disabled = False

    @mainthread
    def _do_stop_button(self):
        """Enables the start button to become a stop button for a simulation."""

        temp_ref = self.ids.startbutton
        temp_ref.background_color = self.red
        temp_ref.text = "STOP"
        temp_ref.disabled = False

    @mainthread
    def _disable_button(self):
        """Resets the start button to default and disables it."""

        temp_ref = self.ids.startbutton
        temp_ref.disabled = True
        temp_ref.background_color = self.green
        temp_ref.text = "START"

    def _start_run(self):
        """Threaded method called to run a simulation."""

        if not self.car_connection.connected:
            self.should_run = False
            self._do_start_button()
            print("Car not connected")
            return
        
        if not self.drone_connection.connected:
            self.should_run = False
            self._do_start_button()
            print("Drone not connected")
            return
        
        # Create sessions called SESSION_NAME running target scripts
        # These are run in tmux sessions to avoid SSH sessions hanging the
        # entire program.
        self.car_connection.create_task(SESSION_NAME, f"python3 {self.settings.get_path('car_file_directory')}{self.settings.get_path('car_file_name')}")
        self.drone_connection.create_task(SESSION_NAME, f"python3 {self.settings.get_path('drone_file_directory')}{self.settings.get_path('drone_file_name')}")
        
        # TODO: Create methods to automate data collection
        # self._data_collect()

        self._do_stop_button()

        # Check if script crashed or were stopped remotely. If so, initiate stop
        while self.should_run:
            car_sessions = self.car_connection.get_tmux_sessions()
            drone_sessions = self.drone_connection.get_tmux_sessions()
            print("Car: ", car_sessions)
            print("Drone:", drone_sessions)
            if (SESSION_NAME not in str(car_sessions) or
                SESSION_NAME not in str(drone_sessions)):

                self._disable_button()
                self.should_run = False
                break

            sleep(0.5)
        
        try:
            self.car_connection.kill_session(SESSION_NAME)
        except:
            print("failed to kill process for car")
        
        try:
            self.drone_connection.kill_session(SESSION_NAME)
        except:
            print("failed to kill process for drone")

        # Stop car wheels
        self.car_connection.create_task("reset", "python3 reset.py")

        self._do_start_button()
