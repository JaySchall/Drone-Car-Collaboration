from res.constants import StyleConstants
from widgets.form import SkyFormFieldMixin
from widgets.form import SkyForm
from widgets.form.formtable import SkyFormTable
from widgets.form.fields import SkyCheckBox, SkySlider

class SimulationSettingsForm(SkyFormTable):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.add_field(SkySlider(label = 'CV Offloading', 
                                 size_hint =  (0.8, None),
                                 size = (1, StyleConstants.cursor_size),
                                 pos_hint = StyleConstants.center_pos,
                                 min = 0,
                                 max = 100,
                                 step= 1))
        self.add_field(SkyCheckBox(label = 'Side Channel'))
        self.add_field(SkyCheckBox(label = 'Logging'))