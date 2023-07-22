from res.constants import ColorConstants, StyleConstants
from widgets.tab import SkyTabbedPannel
from interfaces.settings.settingsform import SettingsForm

from kivy.properties import ObjectProperty
from kivy.metrics import dp
from kivy.lang.builder import Builder

Builder.load_string("""
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
            width: dp(350)
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
                    Button:
                        size_hint: None, None
                        size: root.button_size
                        text: "Defaults"
                        on_release: root.defaults()
                    Button:
                        size_hint: None, None
                        size: root.button_size
                        text: "Apply"
                        on_release: root.apply(setform.form.get_values())
                        
""")


class SkySettingsTab(SkyTabbedPannel):
    def_padding = StyleConstants.def_padding
    settings = ObjectProperty()

    form_bg = ColorConstants.form_bg_color
    form_fg = ColorConstants.form_fg_color
    tab_bg = ColorConstants.tab_bg_color
    button_size = (dp(75), dp(25))

    def apply(self, values):
        form = self.ids["setform"].form
        if form.validate():
            return
        self.settings.set_values(values)

    def defaults(self, *args):
        self.settings.on_default()