from res.constants import ColorConstants, StyleConstants
from widgets.tab import SkyTabbedPannel
from widgets.video import SkyVideoPlayer
from widgets.terminal import SkyTerminal
from widgets.layouts import SkyHorizontalLayout, SkyVerticalLayout
from interfaces.simulation.settings import SimulationSettingsForm
from interfaces.simulation.connect import SkyConnectForm

from threading import Lock, Thread
from time import sleep

from kivy.clock import Clock, mainthread
from kivy.properties import ObjectProperty, StringProperty
from kivy.metrics import dp
from kivy.lang.builder import Builder

SESSION_NAME = 'python'

Builder.load_string("""

<SkySimulationTab>:
    text: 'Simulation'
    SkyHorizontalLayout:
        padding: root.large_padding, root.large_padding
        SkyVerticalLayout:
            padding: root.def_padding, root.def_padding
            spacing: root.def_padding
            size_hint_min_x: root.video_width + root.large_padding
            size_hint: 0.6, 1
            SkyVideoPlayer:
                source: root.link
                video_width: root.video_width
                video_height: root.video_height
            Label:
                id: video_ip
                color: root.black
                size_hint: None, None
                size: self.texture_size
                halign: 'left'
                text: root.link
            SkyTerminal:
        SkyVerticalLayout:
            padding: root.huge_padding, root.huge_padding
            spacing: root.huge_padding
            size_hint_min_x: dp(350)
            size_hint_max_x: dp(600)
            size_hint: 0.4, 1
            SkyVerticalLayout:
                padding: root.huge_padding, root.huge_padding
                spacing: root.huge_padding
                canvas.before:
                    Color:
                        rgba: root.form_bg_color
                    Rectangle:
                        size: self.size
                        pos: self.pos 
                SkyConnectForm:
                    connection: root.drone_connection
                SkyConnectForm:
                    connection: root.car_connection
                SimulationSettingsForm:
                    id: simulation_settings            
            Button:
                id: startbutton
                text: 'START'
                disabled: True
                padding_y: dp(100)
                size_hint: 0.8, 0.2
                pos_hint: root.center_pos
                on_release: root.on_start(simulation_settings.form.get_values())
                background_color: root.green
                font_size: self.height/2
                bold: True
""")

class SkySimulationTab(SkyTabbedPannel):
    
    drone_connection = ObjectProperty()
    car_connection = ObjectProperty()
    link = StringProperty()
    settings = ObjectProperty()
    video_source = StringProperty()

    start_lock = Lock()
    start_thread = None
    
    black = ColorConstants.black
    def_padding = StyleConstants.def_padding
    large_padding = StyleConstants.large_padding
    huge_padding = StyleConstants.huge_padding
    video_width = StyleConstants.video_width
    video_height = StyleConstants.video_height
    green = ColorConstants.green
    red = ColorConstants.red
    center_pos = StyleConstants.center_pos
    form_bg_color = ColorConstants.form_bg_color
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.running_thread = Thread(target=self._start_run)
        self.should_run = False

    def on_start(self, values):
        if not self.running_thread.is_alive():
            self._dissable_button()
            self.should_run = True
            self.running_thread.run()
        else:
            self._dissable_button()
            self.should_run = False

    @mainthread
    def _do_start_button(self):
        temp_ref = self.ids["startbutton"]
        temp_ref.background_color = self.green
        temp_ref.text = "START"
        temp_ref.disabled = False

    @mainthread
    def _do_stop_button(self):
        temp_ref = self.ids["startbutton"]
        temp_ref.background_color = self.red
        temp_ref.text = "STOP"
        temp_ref.disabled = False

    @mainthread
    def _dissable_button(self):
        temp_ref = self.ids["startbutton"]
        temp_ref.disabled = True
        temp_ref.background_color = self.green
        temp_ref.text = "START"

    def _start_run(self):
        if not self.car_connection.connected:
            self.should_run = False
            self._do_start_button()
            print("Car not connected")
            return
        
        while SESSION_NAME not in self.car_connection.open_sessions:
            self.car_connection.create_task(SESSION_NAME, f'python3 {self.settings.get_path("car_file_directory")}{self.settings.get_path("car_file_name")}')
        
        if not self.drone_connection.connected:
            self.should_run = False
            self._do_start_button()
            print("Drone not connected")
            return
        
        while SESSION_NAME not in self.drone_connection.open_sessions:
            self.drone_connection.create_task(SESSION_NAME, f'python3 {self.settings.get_path("drone_file_directory")}{self.settings.get_path("drone_file_name")}')
        
        # self._data_collect()

        self._do_stop_button()
        while self.should_run:
            print("Car: ", self.car_connection.get_tmux_sessions())
            print("Drone:", self.drone_connection.get_tmux_sessions())
            if SESSION_NAME not in str(self.car_connection.get_tmux_sessions()) or SESSION_NAME not in str(self.drone_connection.get_tmux_sessions()):
                self._dissable_button()
                self.should_run = False
                break
            sleep(0.5)
        
        try:
            self.car_connection.kill_session(SESSION_NAME)
        except:
            print('failed to kill process for car')
        
        try:
            self.drone_connection.kill_session(SESSION_NAME)
        except:
            print('failed to kill process for drone')

        self.car_connection.create_task('reset', 'python3 reset.py')

        self._do_start_button()
        

