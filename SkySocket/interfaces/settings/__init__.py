from res.constants import ColorConstants, StyleConstants
from widgets.tab import SkyTabbedPannel
from interfaces.settings.settingsform import SettingsForm

from kivy.uix.button import Button
from kivy.properties import ObjectProperty
from kivy.metrics import dp
from kivy.lang.builder import Builder

Builder.load_string("""
#: import ColorConstants res.constants.ColorConstants
                    
<SettingsButton>:                        
    size_hint: None, None
    size: dp(100), dp(35)
    bold: True
    background_normal: ""
    background_down: ""
    background_color: ColorConstants.button_norm                     

<SkySettingsTab>:
    text: 'Testing'
    AnchorLayout:
        padding: root.def_padding, root.def_padding
        anchor_x: 'left'
        anchor_y: 'top'
        canvas.before:
            Color:
                rgba: root.tab_bg
            Rectangle:
                size: self.size
                pos: self.pos
        SkyVerticalLayout:
            size_hint: None, None
            height: self.minimum_height
            width: dp(500)
            padding: root.def_padding, root.def_padding
            pos_hint: {"top": 1, "left": 0}
            canvas.before:
                Color:
                    rgba: root.form_bg
                Rectangle:
                    size: self.size
                    pos: self.pos
            SkyVerticalLayout:
                pos_hint: {'center_x': 0.5, 'center_y': 0.5}
                padding: root.def_padding, root.def_padding
                spacing: root.def_padding
                size_hint: 1, None
                height: self.minimum_height
                canvas.before:
                    Color:
                        rgba: root.form_fg
                    Rectangle:
                        size: self.size
                        pos: self.pos
                SettingsForm:
                    id: setform
                    settings: root.settings
                StackLayout:
                    orientation: 'rl-bt'
                    size_hint: 1, None
                    height: self.minimum_height
                    spacing: root.def_padding
                    SettingsButton:
                        text: "Apply"
                        on_release: root.apply(setform.form.get_values())
                    SettingsButton:
                        text: "Defaults"
                        on_release: root.defaults()
                        
""")


class SettingsButton(Button):
    def on_press(self):
        self.background_color = ColorConstants.button_down
     
    def on_touch_up(self, *args):
        self.background_color = ColorConstants.button_norm


class SkySettingsTab(SkyTabbedPannel):
    def_padding = StyleConstants.def_padding
    settings = ObjectProperty()

    form_bg = ColorConstants.form_bg_color
    form_fg = ColorConstants.form_fg_color
    tab_bg = ColorConstants.tab_bg_color

    def apply(self, values):
        form = self.ids["setform"].form
        if form.validate():
            return
        self.settings.set_values(values)

    def defaults(self, *args):
        self.settings.on_default()