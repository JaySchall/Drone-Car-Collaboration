from kivy.properties import StringProperty, ObjectProperty, BooleanProperty

class SkyForm:
    uid = StringProperty()
    def __init__(self):
        self._fields = {}

    def add_field(self, field):
        if field.parent_form:
            return
        field.parent_form = self
        self._fields[field.uid] = field

    def get_item(self, id):
        return self._fields[id]
    
    def get_value(self, id):
        return self._fields[id].get_value()
    
    def set_value(self, id, value):
        self._fields[id] = value
    
    def validate(self):
        for fields, values in self._fields:
            values.validate()

class SkyFormFieldMixin:
    uid = StringProperty()
    label = StringProperty()
    value_property = StringProperty()
    parent_form = ObjectProperty()
    is_disabled = BooleanProperty()
    validators = []

    def set_error(self, error):
        if error != True:
            raise NotImplementedError

    def validate(self):
        for validator in self.validators:
            return validator.validate(self.parent_form, self.uid, self.value_property)