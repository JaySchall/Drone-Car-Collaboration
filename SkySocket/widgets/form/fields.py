from kivy.lang.builder import Builder
from kivy.metrics import dp
from kivy.properties import StringProperty

from kivy.uix.textinput import TextInput
from kivy.uix.checkbox import CheckBox
from kivy.uix.slider import Slider

from widgets.form import SkyFormFieldMixin


Builder.load_string("""
<SkyTextInput>:
    id: self.uid
    size_hint: None, None
    height: self.minimum_height
    hint_text: self.input_hint
    text: self.value_property
    on_text: self.value_property = self.text

<SkyCheckBox>:
    id: self.uid
    size_hint: None, None
    size: dp(25), dp(25)
    on_release: self.value_property = self.state


<SkySlider>:
    id: self.uid
    value: self.value_property

""")

class SkyTextInput(TextInput, SkyFormFieldMixin):
    input_hint = StringProperty()
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

class SkyCheckBox(CheckBox, SkyFormFieldMixin):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.value_property = self.state

class SkySlider(Slider, SkyFormFieldMixin):
    def __init__(self, **kwargs):
        self.value_property = '0'
        super().__init__(**kwargs)
