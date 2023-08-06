"""
The logic and backend pertaining to user input forms.
"""

from abc import abstractmethod

from kivy.properties import ListProperty
from kivy.properties import ObjectProperty
from kivy.properties import StringProperty


class SkyForm:
    """
    Holds widgets of a form and controls access to said widgets.

    Attributes:
        _fields: Holds a dictionary of sid-FormFieldWidget pairs.
    """
    
    def __init__(self):

        self._fields = {}

    def add_field(self, field):
        """
        Adds the field the form and assigns the form to the field.

        Args:
            field: A Kivy input widget that inherits the SkyFormFieldMixin.
        """

        # Already in a form
        if field.parent_form:
            return
        
        field.parent_form = self
        self._fields[field.sid] = field

    def get_item(self, sid):
        """
        Getter for an individual field.
        
        Args:
            sid: The unique identifier for the desired input widget.

        Returns:
            SkyFormFieldMixin Kivy widget.
        """

        return self._fields[sid]
    
    def get_items(self):
        """
        Getter for all fields in the form.
        
        Returns:
            Dictionary of sid-SkyFormFieldMixin pairs.
        """

        return self._fields
    
    def get_value(self, sid):
        """
        Getter for a fields current value.

        Args:
            sid: The unique identifier for the desired input widget.
        
        Returns:
            None, bool, str, int, or float.
        """
        
        return self._fields[sid].get_value()
    
    def get_values(self):
        """
        Getter for the values of every field in the form.

        Returns:
            Dictionary of sid-value pairs.
        """

        return {i.sid:i.get_value() for i in self._fields.values()}
    
    def set_value(self, sid, value):
        """
        Setter for an individual field in the form.
        
        Args:
            sid: The unique identifier for the field to be set.
            value: The value to be assigned to the selected field.
        """
        
        self._fields[sid].set_value(value)
    
    def validate(self):
        """
        Calls validate method on every field in the form.

        Returns:
            None or str
        """

        for _, values in self._fields.items():
            val = values.validate()
            if type(val) == str:
                return val


class SkyFormFieldMixin:
    """
    Parent class for any input field to be used in a SkyForm.

    Attributes:
        label: String to be shown next to input widget on the form.
        parent_form: Assigned to the form this widget is added to for reference
            and to ensure it's only on one form.
        sid: A user-defined, unique identifier for reference.
        validators: A list of SkyValidators to be used when validate is called.
    """

    label = StringProperty()
    parent_form = ObjectProperty()
    sid = StringProperty()
    validators = ListProperty()

    @abstractmethod
    def get_value(self):
        """Getter for the value of a SkyFormField."""

        pass

    @abstractmethod
    def set_value(self, value):
        """Setter for the value of a SkyFormField"""

        pass

    def set_error(self, result):
        """
        Raises an error if the field fails it's validation check.
        
        Args:
            result: the returned value from the validator call.
        """

        if result != True:
            raise NotImplementedError

    def validate(self):
        """
        Runs validation for each validator in the field.
        
        Returns:
            str if error in validation else None.
        """
        
        for validator in self.validators:
            return validator.validate(self.parent_form, self.sid,
                                      self.get_value())
