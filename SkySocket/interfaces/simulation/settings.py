"""Creates SkyFormTable for the simulation settings."""

from res.constants import SizeConstants
from res.constants import StyleConstants
from widgets.form import SkyForm
from widgets.form.fields import SkyCheckBox
from widgets.form.fields import SkySlider
from widgets.form.formtable import SkyFormTable

class SimulationSettingsForm(SkyFormTable):
    """Form table widget for simulation settings. See SkyFormTable for more."""

    def __init__(self, **kwargs):
        """Initializes configurations and fields."""

        super().__init__(**kwargs)
        self.form = SkyForm()
        self.add_field(SkySlider(sid="offloading",
                                 label="CV Offloading", 
                                 max=100,
                                 min=0,
                                 pos_hint=StyleConstants.center_pos,
                                 size=(1, SizeConstants.cursor_size),
                                 size_hint=(0.8, None),
                                 step=1), 
                                 height= SizeConstants.slider_height)
        self.add_field(SkyCheckBox(sid="sideChannel", label="Side Channel"))
        self.add_field(SkyCheckBox(sid="Logging", label="Logging"))

        # Special configurations for offloading label to maintain consistency
        offloading_label = self.children[-1].children[-1]
        offloading_label.size_hint = (1, 0.5)
        offloading_label.pos_hint = {"center_y": 0.5}