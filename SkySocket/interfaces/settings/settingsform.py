from widgets.form import SkyForm
from widgets.form.formtable import SkyFormTable
from widgets.form.fields import SkyTextInput

from kivy.metrics import dp

class SettingsForm(SkyFormTable):
    text_width = dp(150)
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.form = SkyForm()
        self.add_field(SkyTextInput(sid = "droneuser", label = "Drone User", width = self.text_width))
        self.add_field(SkyTextInput(sid = "droneip", label = "Drone IP", width = self.text_width))
        self.add_field(SkyTextInput(sid = "dronepass", label = "Drone Password", password=True, width = self.text_width))
        self.add_field(SkyTextInput(sid = "caruser", label = "Car User", width = self.text_width))
        self.add_field(SkyTextInput(sid = "carip", label = "Car IP", width = self.text_width))
        self.add_field(SkyTextInput(sid = "carpass", label = "Car Password", password=True, width = self.text_width))
        self.add_field(SkyTextInput(sid = "videosource", label = "Video Source", width = self.text_width))
        self.add_field(SkyTextInput(sid = "offloading", label = "Offloading Default", width = self.text_width))
        self.add_field(SkyTextInput(sid = "log", label = "Log Directory", width = self.text_width))
        self.add_field(SkyTextInput(sid = "results", label = "Results Directory", width = self.text_width))
        self.add_field(SkyTextInput(sid = "dronefile", label = "Drone File Directory", width = self.text_width))
        self.add_field(SkyTextInput(sid = "dronename", label = "Drone File Name", width = self.text_width))
        self.add_field(SkyTextInput(sid = "carfile", label = "Car File Directory", width = self.text_width))
        self.add_field(SkyTextInput(sid = "carname", label = "Car File Name", width = self.text_width))
