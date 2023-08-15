"""The backend logic for the actual settins form."""

from kivy.metrics import dp
from kivy.metrics import sp
from kivy.properties import ObjectProperty

from res.constants import SizeConstants
from widgets.form import SkyForm
from widgets.form.fields import SkyTextInput
from widgets.form.formtable import SkyFormTable
from widgets.form.validator import SkyNetworkValidator
from widgets.form.validator import SkyPathValidator


class SkySettingsTextInput(SkyTextInput):
    """A special visual configuration for all text inputs on the settings tab."""
    
    def __init__(self, **kwargs):

        super().__init__(font_size=SizeConstants.settings_text_height/2, 
                         height=SizeConstants.settings_text_height,
                         size_hint=(1.5, None), **kwargs)

class SettingsIDS():

    DRONE_USERNAME = "drone_username"
    DRONE_IP = "drone_ip"
    DRONE_PASSWORD = "drone_password"
    CAR_USERNAME = "car_username"
    CAR_IP = "car_ip"
    CAR_PASSWORD = "car_password"
    VIDEO_SOURCE = "video_source"
    OFFLOADING = "offloading_percent"
    LOG = "log_directory"
    RESULTS = "results_directory"
    DRONE_DIRECTORY = "drone_file_directory"
    DRONE_NAME = "drone_file_name"
    CAR_DIRECTORY = "car_file_directory"
    CAR_NAME = "car_file_name"


class SettingsForm(SkyFormTable):
    """
    The form table for the settings tab.
    
    Attributes:
        text_width: The default width for the settings labels to be.
        settings: The SkyVariables object for access to the config.
    """

    settings = ObjectProperty()
    text_width = dp(150)

    def __init__(self, **kwargs):
        """Initializes the form with all fields."""

        super().__init__(row_height=SizeConstants.settings_text_height, **kwargs)
        
        _ip_val = SkyNetworkValidator(ip=True)
        _local_dir = SkyPathValidator(local_path=True)

        self.form = SkyForm()
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.DRONE_USERNAME, label="Drone User", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.DRONE_IP, label = "Drone IP", width = self.text_width, validators=[_ip_val]))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.DRONE_PASSWORD, label="Drone Password", password=True, width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.CAR_USERNAME, label="Car User", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.CAR_IP, label="Car IP", width=self.text_width, validators=[_ip_val]))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.CAR_PASSWORD, label="Car Password", password=True, width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.VIDEO_SOURCE, label="Video Source", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.OFFLOADING, label="Offloading Default", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.LOG, label="Log Directory", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.RESULTS, label="Results Directory", width=self.text_width, validators=[_local_dir]))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.DRONE_DIRECTORY, label="Drone File Directory", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.DRONE_NAME, label="Drone File Name", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.CAR_DIRECTORY, label="Car File Directory", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid=SettingsIDS.CAR_NAME, label="Car File Name", width=self.text_width))

    def on_kv_post(self, base_widget):

        self.form.set_value(SettingsIDS.DRONE_USERNAME, self.settings[SettingsIDS.DRONE_USERNAME])
        self.form.set_value(SettingsIDS.DRONE_IP, self.settings[SettingsIDS.DRONE_IP])
        self.form.set_value(SettingsIDS.DRONE_PASSWORD, self.settings[SettingsIDS.DRONE_PASSWORD])
        self.form.set_value(SettingsIDS.CAR_USERNAME, self.settings[SettingsIDS.CAR_USERNAME])
        self.form.set_value(SettingsIDS.CAR_IP, self.settings[SettingsIDS.CAR_IP])
        self.form.set_value(SettingsIDS.CAR_PASSWORD, self.settings[SettingsIDS.CAR_PASSWORD])
        self.form.set_value(SettingsIDS.VIDEO_SOURCE, self.settings[SettingsIDS.VIDEO_SOURCE])
        self.form.set_value(SettingsIDS.OFFLOADING, self.settings[SettingsIDS.OFFLOADING])
        self.form.set_value(SettingsIDS.LOG, self.settings[SettingsIDS.LOG])
        self.form.set_value(SettingsIDS.RESULTS, self.settings[SettingsIDS.RESULTS])
        self.form.set_value(SettingsIDS.DRONE_DIRECTORY, self.settings[SettingsIDS.DRONE_DIRECTORY])
        self.form.set_value(SettingsIDS.DRONE_NAME, self.settings[SettingsIDS.DRONE_NAME])
        self.form.set_value(SettingsIDS.CAR_DIRECTORY, self.settings[SettingsIDS.CAR_DIRECTORY])
        self.form.set_value(SettingsIDS.CAR_NAME, self.settings[SettingsIDS.CAR_NAME])

        super().on_kv_post(base_widget)