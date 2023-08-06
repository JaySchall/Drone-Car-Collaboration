from kivy.lang.builder import Builder
from kivy.uix.button import Button

from res.constants import ColorConstants

Builder.load_string("""
#: import SizeConstants res.constants.SizeConstants

<SkyButton>:
    background_color: self.background_norm    
    background_down: ""       
    background_normal: ""       
    bold: True
    size: SizeConstants.button_size
    size_hint: None, None
""")

class SkyButton(Button):
    """
    A visually styled button.
    
    Attributes:
        background_norm: The default background color for the button.
    """

    background_norm = ColorConstants.button_norm

    def on_press(self):
        """Change button color when pressed."""

        self.background_color = ColorConstants.button_down
     
    def on_touch_up(self, *args):
        """Change button color when mouse releases."""

        self.background_color = ColorConstants.button_norm