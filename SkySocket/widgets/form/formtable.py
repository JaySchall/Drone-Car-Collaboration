from kivy.lang.builder import Builder
from kivy.metrics import dp
from kivy.properties import (StringProperty, NumericProperty,
                             BooleanProperty, ObjectProperty)

from kivy.uix.textinput import TextInput
from kivy.uix.checkbox import CheckBox
from kivy.uix.slider import Slider
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout

from widgets.layouts import SkyVerticalLayout
from res.constants import ColorConstants
from widgets.form import SkyFormFieldMixin, SkyForm
from widgets.form.validator import (SkyPathValidator,SkyNetworkValidator, 
                                    SkyRequiredValidator, SkyNumberValidator)



Builder.load_string("""
<SkyFormTable>:
    size_hint: 1, 1
    height: self.minimum_height
    padding: dp(5), dp(5)
    spacing: dp(5)

<SkyFormLabel>:
    size_hint: 1, 1
    color: self.black
    text_size: self.size
""")

class SkyFormLabel(Label):
    black = ColorConstants.black

class SkyFormTable(SkyVerticalLayout):
    form = ObjectProperty()
    row_height = NumericProperty(dp(20))
    field_width = NumericProperty(dp(200))
    label_width = NumericProperty(dp(100))
    fill_width = BooleanProperty(False)
    label_kwargs = {}
    form_field_kwargs = {}

    def add_fields(self, fields):
        for item in fields:
            self.add_field(item)
    
    def add_field(self, field):
        #self.form.add_field(field)
        row = BoxLayout(
            orientation='horizontal',
            size_hint = (1, 1),
            height=self.row_height
        )
        row.add_widget(SkyFormLabel(text=field.label))
        if self.fill_width:
            row.add_widget(Label(size_hint=(1, 1)))
        row.add_widget(field)
        self.add_widget(row)
    
    def validate(self):
        SkyForm.validate()
