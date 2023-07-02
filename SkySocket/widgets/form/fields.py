from kivy.lang.builder import Builder
from kivy.metrics import dp
from kivy.properties import StringProperty

import win32ui
import win32con

from kivy.properties import ColorProperty
from kivy.uix.textinput import TextInput
from kivy.uix.checkbox import CheckBox
from kivy.uix.slider import Slider
from kivy.uix.button import Button

from res.constants import ColorConstants
from widgets.form import SkyFormFieldMixin

EXTENSIONS = "All Files |*.*|"

Builder.load_string("""
<SkyTextInput>:
    id: self.uid
    multiline: False
    size_hint: None, None
    height: self.minimum_height
    padding: dp(4), dp(4)
    hint_text: self.input_hint
    font_size: sp(10)
    on_text: self.value_property = self.text

<SkyCheckBox>:
    id: self.uid
    size_hint: None, None
    size: dp(25), dp(25)
    on_release: self.value_property = self.state

<SkySlider>:
    id: self.uid
    min: 0
    max: 100

<SkyBrowse>:
    id: self.uid
    color: self.black
    size_hint: None, None
    background_normal: ''
    background_color: self.button_bg
    label: self.returned
    on_release: self.browse_save()
""")

class SkyTextInput(TextInput, SkyFormFieldMixin):
    input_hint = StringProperty()
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def get_value(self):
        return self.text
    
    def set_value(self, value):
        self.text=value


class SkyCheckBox(CheckBox, SkyFormFieldMixin):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
    def get_value(self):
        return self.active
    
    def set_value(self, value):
        self.active = value


class SkySlider(Slider, SkyFormFieldMixin):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.value = 0

    def get_value(self):
        return self.value
    
    def set_value(self, value):
        self.value = value


class SkyBrowse(Button, SkyFormFieldMixin):
    button_bg = ColorProperty(ColorConstants.tab_bg_color)
    black = ColorProperty(ColorConstants.black)
    returned = StringProperty("")

    def browse_save(self):
        flag_choices = win32con.OFN_HIDEREADONLY|win32con.OFN_OVERWRITEPROMPT
        dlg = win32ui.CreateFileDialog(0, None, None, flag_choices, EXTENSIONS)
        dlg.SetOFNTitle("Save Location")
        if dlg.DoModal() == win32con.IDOK:
            self.returned = dlg.GetFileName()

    def get_value(self):
        return self.returned
    
    def browse_open(self):
        pass