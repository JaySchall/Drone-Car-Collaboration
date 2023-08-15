"""
Input widgets with features and custom configuration for Sky Socket.

Class:
    SkyTextInput
    SkyCheckBox
    SkySlider
    SkyBrowse
"""

from kivy.lang.builder import Builder
from kivy.properties import StringProperty
from kivy.uix.button import Button
from kivy.uix.checkbox import CheckBox
from kivy.uix.slider import Slider
from kivy.uix.textinput import TextInput

import win32con
import win32ui


from widgets.form import SkyFormFieldMixin

EXTENSIONS = "All Files |*.*|"

Builder.load_string("""
#: import ColorConstants res.constants.ColorConstants                

<SkyTextInput>:
    id: self.uid
    font_size: sp(10)
    height: self.minimum_height
    multiline: False
    padding: dp(4), dp(4)
    size_hint: None, None

<SkyCheckBox>:
    id: self.uid
    height: dp(20)
    size_hint: None, None
    width: self.height

<SkySlider>:
    id: self.uid
    max: 100
    min: 0
    value: 0

<SkyBrowse>:
    id: self.uid
    background_color: ColorConstants.tab_bg_color
    background_down: ""
    background_normal: ""
    color: ColorConstants.black
    label: self.returned
    size_hint: None, None
    on_right_click: self.browse_save()
""")


class SkyTextInput(TextInput, SkyFormFieldMixin):
    """Text input with getters/setters attached to the text property."""

    def get_value(self):

        return self.text
    
    def set_value(self, value):
        
        self.text=value


class SkyCheckBox(CheckBox, SkyFormFieldMixin):
    """Checkbox input with getters/setters attached to the active property."""

    def get_value(self):

        return self.active
    
    def set_value(self, value):

        self.active = value


class SkySlider(Slider, SkyFormFieldMixin):
    """Slider input with getters/setters attached to the value property."""

    def get_value(self):

        return self.value
    
    def set_value(self, value):

        self.value = value


class SkyBrowse(Button, SkyFormFieldMixin):
    """
    A custom button used for file selection.
    
    This class is not used in the project yet but is implemented for future use.
    Attributes:
        returned: The currently selected path.
    """

    returned = StringProperty("")

    def browse_save(self, *args):
        """
        Called when the user presses the 'Browse...' button.
        
        Opens a win32 file dialog for the user to select an option. For more
        information about the OFN flags, see windows documentation on open file
        flags. The CreateFileDialog call creates a file chooser dialog (0), uses
        the selected flags, and gives the user the option to choose from the
        EXTENSIONS set of file extensions.
        """

        flag_choices = win32con.OFN_HIDEREADONLY|win32con.OFN_OVERWRITEPROMPT
        dlg = win32ui.CreateFileDialog(0, None, None, flag_choices, EXTENSIONS)
        dlg.SetOFNTitle("Save Location")

        # Renders the dialog and only checks result if user presses "OK"
        if dlg.DoModal() == win32con.IDOK:
            self.returned = dlg.GetFileName()

    def get_value(self):

        return self.returned
    
    def set_value(self, value):
        
        self.returned = value
