from kivy.uix.tabbedpanel import TabbedPanelItem
from kivy.lang.builder import Builder

CLICKED_COLOR = [0.85, 0.85, 0.85, 1]
UNCLICKED_COLOR = [0.74, 0.74, 0.74, 1]

Builder.load_string("""
<SkyTabbedPannel>:
    color: [0, 0, 0, 1]
    background_disabled_down: ''
    background_disabled_normal: ''
    background_down: ''
    background_normal: ''
""")

class SkyTabbedPannel(TabbedPanelItem):
    
    def on_state(self, *args):
        if args[1] == 'normal':
            self.background_color = UNCLICKED_COLOR
        else:
            self.background_color = CLICKED_COLOR

