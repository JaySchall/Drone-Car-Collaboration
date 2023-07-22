from kivy.properties import StringProperty, ObjectProperty, BooleanProperty
from kivy.properties import ListProperty

class SkyForm:
    def __init__(self):
        self._fields = {}

    def add_field(self, field):
        if field.parent_form:
            return
        field.parent_form = self
        self._fields[field.sid] = field

    def get_item(self, id):
        return self._fields[id]
    
    def get_items(self):
        return self._fields
    
    def get_value(self, id):
        return self._fields[id].get_value()
    
    def get_values(self):
        return {i.sid:i.get_value() for i in self._fields.values()}
    
    def set_value(self, id, value):
        self._fields[id].set_value(value)
    
    def validate(self):
        for fields, values in self._fields.items():
            val = values.validate()
            if type(val) == str:
                return val

class SkyFormFieldMixin:
    sid = StringProperty()
    label = StringProperty()
    parent_form = ObjectProperty()
    is_disabled = BooleanProperty()
    validators = ListProperty()

    def set_error(self, error):
        if error != True:
            raise NotImplementedError

    def validate(self):
        for validator in self.validators:
            return validator.validate(self.parent_form, self.sid, self.value_property)