"""
A video widget for displaying the drone's camera feed.
    
This widget allows users to see the camera feeds they desire from the drone to
monitor the system during research.

The SkyVideoPlayer widget is not fully developed yet. It still requires:
    - Reconnection capabilities.
    - Consistent discoverability.
    - Play/Pause feature while playing.
"""

from kivy.lang.builder import Builder
from kivy.uix.video import Video

Builder.load_string("""
#: import SizeConstants res.constants.SizeConstants
#: import StyleConstants res.constants.StyleConstants
                    
<SkyVideoPlayer>:
    id: video
    allow_stretch: True
    keep_ratio: True
    pos_hint: StyleConstants.center_pos
    size: SizeConstants.video_width, SizeConstants.video_height
    size_hint_min_y: SizeConstants.video_height
    state: "stop"
    Button:
        size: root.size
        pos: root.pos
        on_release: root._state()
        color: [0, 0, 0, 0]
        border: [0, 0, 0 ,0]
        background_normal: ""
        background_down: ""
        background_disabled_normal: ""
        background_disabled_down: ""
""")


class SkyVideoPlayer(Video):
    """
    Displays the video feed from the drone's camera.

    This widget displays video from a predefined URL from the drone's server.
    The link is passed form the settings widget and is configurable on the
    settings page. 
    """
    
    def _state(self):
        self.reload()
        self.state = "play" if self.state == "stop" else "stop"