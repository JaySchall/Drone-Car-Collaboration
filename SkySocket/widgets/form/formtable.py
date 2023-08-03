from kivy.lang.builder import Builder
from kivy.metrics import dp
from kivy.properties import (NumericProperty, BooleanProperty, ObjectProperty)


from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout

from widgets.layouts import SkyVerticalLayout
from res.constants import ColorConstants
from widgets.form import SkyForm


Builder.load_string("""
<SkyFormTable>:
    size_hint: 1, None
    height: self.minimum_height
    padding: dp(5), dp(5)
    spacing: dp(5)

<SkyFormLabel>:
    color: self.black
    text_size: self.size
    font_size: sp((self.size[1]/5) + 12)
    valign: "center"
""")

class SkyFormLabel(Label):
    black = ColorConstants.black

class SkyFormTable(SkyVerticalLayout):
    form = ObjectProperty()
    row_height = NumericProperty(dp(20))
    field_width = NumericProperty(dp(200))
    label_width = NumericProperty(dp(100))
    fill_width = BooleanProperty(False)

    def add_fields(self, fields):
        for item in fields:
            self.add_field(item)
    
    def add_field(self, field, **kwargs):
        self.form.add_field(field)
        if "height" not in kwargs:
            kwargs["height"] = self.row_height
        if "size_hint" not in kwargs:
            kwargs["size_hint"] = (1, None)
        row = BoxLayout(
            orientation='horizontal',
            **kwargs
        )

        row.add_widget(SkyFormLabel(text=field.label))
        if self.fill_width:
            row.add_widget(Label(size_hint=(1, 1)))
        row.add_widget(field)
        self.add_widget(row)
    
    def validate(self):
        SkyForm.validate()