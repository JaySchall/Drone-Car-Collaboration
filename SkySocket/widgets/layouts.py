from kivy.uix.boxlayout import BoxLayout
from kivy.uix.stacklayout import StackLayout
from kivy.lang.builder import Builder

Builder.load_string("""
<SkyHorizontalLayout>:
    orientation: 'horizontal'

<SkyVerticalLayout>:
    orientation: 'vertical'

<SkyStackLayout>:
    orientation: 'lr-tb'
""")

class SkyHorizontalLayout(BoxLayout):
    pass

class SkyVerticalLayout(BoxLayout):
    pass

class SkyStackLayout(StackLayout):
    pass