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
        self.add_field(SkySettingsTextInput(sid="drone_username", label="Drone User", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="drone_ip", label = "Drone IP", width = self.text_width, validators=[_ip_val]))
        self.add_field(SkySettingsTextInput(sid="drone_password", label="Drone Password", password=True, width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="car_username", label="Car User", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="car_ip", label="Car IP", width=self.text_width, validators=[_ip_val]))
        self.add_field(SkySettingsTextInput(sid="car_password", label="Car Password", password=True, width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="video_source", label="Video Source", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="offloading_percent", label="Offloading Default", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="log_directory", label="Log Directory", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="results_directory", label="Results Directory", width=self.text_width, validators=[_local_dir]))
        self.add_field(SkySettingsTextInput(sid="drone_file_directory", label="Drone File Directory", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="drone_file_name", label="Drone File Name", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="car_file_directory", label="Car File Directory", width=self.text_width))
        self.add_field(SkySettingsTextInput(sid="car_file_name", label="Car File Name", width=self.text_width))
