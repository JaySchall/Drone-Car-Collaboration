from kivy.lang.builder import Builder
from kivy.uix.label import Label

from res.constants import StyleConstants, ColorConstants

Builder.load_string("""
<SkyTerminal>:
    id: terminal_placeholder
    text: "TERMINAL PLACEHOLDER"
    bold: True
    size_hint: 1, 0.2
    pos_hint: self.center_pos
    padding: dp(40), dp(40)
    canvas.before:
        Color:
            rgba: self.bg_color
        Rectangle:
            size: self.size
            pos: self.pos
""")

class SkyTerminal(Label):
    center_pos = StyleConstants.center_pos
    bg_color = ColorConstants.blue