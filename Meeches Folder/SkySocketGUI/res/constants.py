"""Contains classes of constant values for use throughout the program."""

from kivy.metrics import dp
from kivy.metrics import sp


class ColorConstants:
    """Color value lists."""

    black = [0, 0, 0, 1]
    blue = [0, 0, 1, 1]
    green = [0, 1, 0.5, 1]
    orange = [1, 0.75, 0, 1]
    red = [1, 0.4, 0.4, 1]
    transparent = [0, 0, 0, 0]
    white = [1, 1, 1, 1]
    
    form_bg_color = [0.8, 0.8, 0.8, 1]
    form_fg_color = [0.93,0.93,0.93,1]
    tab_bg_color = [0.85, 0.85, 0.85, 1]
    tab_bg_clicked_color = [0.74, 0.74, 0.74, 1]

    button_down = [0.5, 0.5, 0.5, 1]
    button_down_disabled = [0.6, 0.6, 0.6, 1]
    button_norm = [0.62, 0.62, 0.62, 62]
    button_norm_disabled = [0.7, 0.7, 0.7, 1]


class SizeConstants:
    """Constants for widget sizes."""

    button_size = (dp(100), dp(35))
    connect_min_size = dp(350)
    connect_max_size = dp(600)
    cursor_size = sp(30)
    min_start = dp(120)
    settings_text_height = dp(30)
    settings_width = dp(500)
    slider_height = dp(40)
    spacer_size = dp(20)
    tab_height = dp(30)
    tab_width = dp(150)
    video_width = dp(640)
    video_height = dp(360)

class StyleConstants:
    """Styling and size values."""

    def_padding = dp(5)
    large_padding = dp(10)
    huge_padding = dp(15)
    terminal_padding = dp(40)
    start_padding = dp(100)
    center_pos = {"center_x": 0.5, "center_y": 0.5}


def calculate_size(size):
    """Dynamically calculates text size on resize."""

    return (size/5) + 12