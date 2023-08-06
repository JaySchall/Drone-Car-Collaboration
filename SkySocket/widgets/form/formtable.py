"""
Contains the logic for rendering the logical SkyForm into a widget.
"""

from kivy.lang.builder import Builder
from kivy.metrics import dp
from kivy.properties import NumericProperty
from kivy.properties import ObjectProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label

from widgets.form import SkyForm
from widgets.layouts import SkyVerticalLayout

Builder.load_string("""
#: import calculate_size res.constants.calculate_size
#: import ColorConstants res.constants.ColorConstants
#: import StyleConstants res.constants.StyleConstants
                    
<SkyFormLabel>:
    color: ColorConstants.black
    font_size: sp(calculate_size(self.size[1]))
    text_size: self.size
    valign: "center"
                    
<SkyFormTable>:
    height: self.minimum_height
    padding: StyleConstants.def_padding, StyleConstants.def_padding
    size_hint: 1, None
    spacing: StyleConstants.def_padding
""")


class SkyFormLabel(Label):
    """A custom stylized label for form inputs."""

    pass


class SkyFormTable(SkyVerticalLayout):
    """
    The 'front-end' of form functionality.

    Add field widgets to the internal form and render said fields into a unified
    layout for use in the GUI.

    Attributes:
        form: A SkyForm for data storage.
        row_height: Defines the default height of each row.
    """
    
    form = ObjectProperty()
    row_height = NumericProperty(dp(20))

    def add_fields(self, fields):
        """
        Add multiple fields to a the form table.
        
        Args:
            fields: A list of SkyFormInputMixin descendants.
        """

        for item in fields:
            self.add_field(item)
    
    def add_field(self, field, **kwargs):
        """
        Add a single field to the form table.
        
        Args:
            field: The SkyFormInputMixin descendant to add to the form.
            kwargs: A dictionary of properties to be passed to the rows layout.
        """

        self.form.add_field(field)

        # Default value check
        if "height" not in kwargs:
            kwargs["height"] = self.row_height
        if "size_hint" not in kwargs:
            kwargs["size_hint"] = (1, None)

        row = BoxLayout(orientation="horizontal", **kwargs)
        row.add_widget(SkyFormLabel(text=field.label))
        row.add_widget(field)
        
        self.add_widget(row)
    
    def validate(self):
        """Initiates validation chain."""

        SkyForm.validate()