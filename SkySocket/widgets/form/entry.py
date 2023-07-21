from kivy.properties import AliasProperty, StringProperty
from kivy.lang.builder import Builder
from kivy.metrics import dp
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.behaviors.focus import FocusBehavior

from widgets.form.fields import SkyTextInput, SkyCheckBox
from res.constants import ColorConstants, StyleConstants

Builder.load_string("""

<SkyFieldBase>:
    orientation: 'horizontal'
    size_hint: 1, None
    height: self.minimum_height
    Label:
        color: root.black
        text: root.label_text
        size_hint: None, None
        size: self.texture_size
        pos_hint: {'x': 0, 'center_y': 0.5}
    Label:
        size_hint: 1, None
        height: 0

<SkyTextField>:
    input_width: self.settings_width
    spacing: self.huge_padding
    SkyTextInput:
        width: root.input_width
        halign: 'right'
        pos_hint: {"right":1,"center_y":0.5}
        hint_text: root.hint
        multiline: False
        on_focus: if not self.focus: self.parent.validate_input(*args) 

<SkyCheckBoxField>:
    SkyCheckBox:
        pos_hint: {'x': 0, 'center_y': 0.5}
""")

class SkyFieldBase(BoxLayout):
    black = ColorConstants.black
    huge_padding = StyleConstants.huge_padding
    label_text = StringProperty("Default Label")
    input_width = dp(100)
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def validate_input(self, instance):
        print(instance)

    def get_value(self):
        return self._text_input.get_value()
    
    def set_value(self, v):
        self._text_input.set_value(v)
    
    def validate(self):
        pass

class SkyTextField(SkyFieldBase):
    hint = StringProperty("Type here...")
    settings_width = StyleConstants.settings_width
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def validate_input(self, *args, **kwargs):
        print(args, kwargs)

class SkyCheckBoxField(SkyFieldBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def validate_input(self, *args, **kwargs):
        print(args, kwargs)