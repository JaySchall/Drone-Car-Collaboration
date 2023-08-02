from kivy.lang.builder import Builder
from kivy.uix.video import Video
from kivy.properties import NumericProperty

from res.constants import StyleConstants

Builder.load_string("""
<SkyVideoPlayer>:
    id: video
    state: 'play' if self.loaded else 'stop'
    keep_ratio: True
    allow_stretch: True
    size_hint_min_y: self.video_length
    pos_hint: self.center_pos
    size: self.video_width, self.video_length
""")

class SkyVideoPlayer(Video):
    center_pos = StyleConstants.center_pos
    video_width = NumericProperty()
    video_length = NumericProperty()