from kivy.uix.tabbedpanel import TabbedPanelItem
from kivy.lang.builder import Builder

from res.constants import ColorConstants

Builder.load_string("""
<SkyTabbedPannel>:
    color: self.black
    canvas.before:
        Color:
            rgba: self.bg_color
        Rectangle:
            size: self.size
            pos: self.pos  
""")

class SkyTabbedPannel(TabbedPanelItem):
    black = ColorConstants.black
    bg_color = ColorConstants.form_bg_color