"""A settings panel to allow for users to change the default parameters."""

from kivy.lang.builder import Builder
from kivy.properties import ObjectProperty
from kivy.uix.button import Button

from interfaces.settings.settingsform import SettingsForm
from res.constants import ColorConstants
from widgets.tab import SkyTabbedPanel

Builder.load_string("""
#: import ColorConstants res.constants.ColorConstants
#: import SizeConstants res.constants.SizeConstants
#: import StyleConstants res.constants.StyleConstants
                    
<SettingsButton>:
    background_color: ColorConstants.button_norm    
    background_down: ""       
    background_normal: ""       
    bold: True
    size: SizeConstants.button_size
    size_hint: None, None        

<SkySettingsTab>:
    text: "Settings"
    AnchorLayout:
        anchor_x: "left"
        anchor_y: "top"
        padding: StyleConstants.def_padding, StyleConstants.def_padding
        canvas.before:
            Color:
                rgba: ColorConstants.tab_bg_color
            Rectangle:
                pos: self.pos
                size: self.size
        SkyVerticalLayout:
            height: self.minimum_height
            padding: StyleConstants.def_padding, StyleConstants.def_padding
            pos_hint: {"top": 1, "left": 0}
            size_hint: None, None
            width: SizeConstants.settings_width
            canvas.before:
                Color:
                    rgba: ColorConstants.form_bg_color
                Rectangle:
                    pos: self.pos
                    size: self.size
            SkyVerticalLayout:
                height: self.minimum_height
                padding: StyleConstants.def_padding, StyleConstants.def_padding
                pos_hint: StyleConstants.center_pos
                size_hint: 1, None
                spacing: StyleConstants.def_padding
                canvas.before:
                    Color:
                        rgba: ColorConstants.form_fg_color
                    Rectangle:
                        pos: self.pos
                        size: self.size
                SettingsForm:
                    id: setform
                    settings: root.settings
                StackLayout:
                    height: self.minimum_height
                    orientation: "rl-bt"
                    size_hint: 1, None
                    spacing: StyleConstants.def_padding
                    SettingsButton:
                        text: "Apply"
                        on_release: root.apply(setform.form.get_values())
                    SettingsButton:
                        text: "Defaults"
                        on_release: root.defaults()
""")


class SettingsButton(Button):
    """Stylized button for settings"""

    def on_press(self, *args):
        """Change button color when pressed."""

        super().on_press()
        self.background_color = ColorConstants.button_down
     
    def on_touch_up(self, *args):
        """Change button color when mouse releases."""

        super().on_touch_up(*args)
        self.background_color = ColorConstants.button_norm


class SkySettingsTab(SkyTabbedPanel):
    """
    The layout and button logic for the settings menu.
    
    Attributes:
        settings: SkyVariables object to apply and read settings from.
    """

    settings = ObjectProperty()

    def apply(self, values):
        """
        Validates the form and writes the values to the config file.
        
        Args:
            values: A dictionary of field-value pairs of the requested setttings.
        """

        form = self.ids.setform.form
        if form.validate():
            return
        self.settings.set_values(values)

    def defaults(self, *args):
        """Resets all settings to default."""

        self.settings.on_default()