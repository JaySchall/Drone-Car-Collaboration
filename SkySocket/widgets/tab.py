"""
Defines standard panel styling.
"""

from kivy.lang.builder import Builder
from kivy.uix.tabbedpanel import TabbedPanelItem

Builder.load_string("""
#: import ColorConstants res.constants.ColorConstants

<SkyTabbedPanel>:
    color: ColorConstants.black
    canvas.before:
        Color:
            rgba: ColorConstants.form_bg_color
        Rectangle:
            pos: self.pos  
            size: self.size
""")


class SkyTabbedPanel(TabbedPanelItem):
    """See TabbedPanelItem documentation from Kivy for more information."""
    
    pass
