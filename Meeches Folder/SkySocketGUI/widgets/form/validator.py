"""
A base for form field validation along with some custom validators.

Classes:
    SkyValidatorMixin
    SkyRequiredValidator
    SkyPathValidator
    SkyNetworkValidator
    SkyNumberValidator
"""

from abc import ABC
from abc import abstractmethod
from os import path
from re import fullmatch

from kivy.event import EventDispatcher
from kivy.properties import NumericProperty
from kivy.properties import BooleanProperty


class SkyValidatorMixin(ABC):
    """Base class for a validator."""

    @abstractmethod
    def validate(self, field, value):
        "Base validation method. To be overwritten."

        pass


class SkyRequiredValidator(SkyValidatorMixin, EventDispatcher):
    """Validator for a required field"""

    def validate(self, field, value):
        """
        Validates that the field has a value.
        
        Returns:
            str: an error string if value is None
            True: If the value is anything but None.
        """

        if value is not None:
            return True
        else:
            return f"{field} is required."


class SkyNetworkValidator(SkyValidatorMixin, EventDispatcher):
    """
    Validator for networking form inputs.
    
    Attributes:
        ip: True to validate the input as a valid IP address.
        port: True to validate the input as a valid port.
    """

    ip = BooleanProperty(False)
    port = BooleanProperty(False)

    def validate(self, field, value):
        """Calls the relevant validating method."""

        if self.port:
            ret_val = self._port(field, value)
            if type(ret_val) == str:
                return ret_val
            
        if self.ip:
            ret_val = self._ip(field, value)
            if type(ret_val) == str:
                return ret_val
            
        return True

    def _port(self, field, value):
        """
        Checks to see if the value is numeric and between 0 and 65535
                
        Returns:
            str: an error string if value is invalid
            True: If the value is valid.
        """

        if not value.isnumeric():
            return f"{value} is not a port number."
        
        if int(value) < 0 or int(value) > 65535:
            return f"{value} is not a valid port number."
        
        return True

    def _ip(self, field, value):
        """
        Checks to see if the value is in port form.
                
        Returns:
            str: an error string if value is invalid
            True: If the value is valid.
        """

        if not fullmatch(r"[0-9]?[0-9]?[0-9]\.[0-9]?[0-9]?[0-9]\.[0-9]?[0-9]?[0-9].[0-9]?[0-9]?[0-9]",
                         value):
            return f"{value} is not in ipv4 format."
        
        for i in value.split("."):
            if int(i) < 0 or int(i) > 255:
                return f"{value} is not a valid ip address"
            
        return True


class SkyNumberValidator(SkyValidatorMixin, EventDispatcher):
    """
    Validator for file paths.
    
    Attributes:
        integer: True if the value should be an integer.
        gt: A number that the value should be greater than.
        gte: A number that the value should be greater than or equal to.
        lt: A number that the value should be less than.
        lte: A number that the value should be less than or eqaul to.
    """

    integer = BooleanProperty(False)
    gt = NumericProperty(None)
    gte = NumericProperty(None)
    lt = NumericProperty(None)
    lte = NumericProperty(None)

    def validate(self, field, value):
        """Calls the relevant validating method(s)."""

        if self.integer:
            ret_val = self._integer(field, value)
            if type(ret_val) == str:
                return ret_val
            
        if self.gt is not None:
            ret_val = self._gt(field, value)
            if type(ret_val) == str:
                return ret_val
            
        if self.gte is not None:
            ret_val = self._gte(field, value)
            if type(ret_val) == str:
                return ret_val
            
        if self.lt is not None:
            ret_val = self._lt(field, value)
            if type(ret_val) == str:
                return ret_val
            
        if self.lte is not None:
            ret_val = self._lte(field, value)
            if type(ret_val) == str:
                return ret_val
            
        return True
        
    def _integer(self, field, value):
        """
        Checks to see if the value is an integer.
                
        Returns:
            str: an error string if value is invalid
            True: If the value is valid.
        """

        if not value.isdigit():
            return f"{value} is not an integer."
        return True

    def _gt(self, field, value):
        """
        Checks to see if the value is greater than the predefined number.
                
        Returns:
            str: an error string if value is invalid
            True: If the value is valid.
        """

        if self._integer and int(value) <= self.gt:
            return f"{value} is not greater than {self.gt}."
        return True

    def _gte(self, field, value):
        """
        Checks to see if the value is greater than or equal to the predefined
        number.
                
        Returns:
            str: an error string if value is invalid
            True: If the value is valid.
        """

        if self._integer and int(value) < self.gte:
            return f"{value} is not greater than or equal to {self.gte}."
        return True

    def _lt(self, field, value):
        """
        Checks to see if the value is less than the predefined number.
                
        Returns:
            str: an error string if value is invalid
            True: If the value is valid.
        """

        if self._integer and int(value) >= self.lt:
            return f"{value} is not less than {self.lt}."
        return True
    
    def _lte(self, field, value):
        """
        Checks to see if the value is less than or equal to the predefined 
        number.
                
        Returns:
            str: an error string if value is invalid
            True: If the value is valid.
        """
                
        if self._integer and int(value) > self.lte:
            return f"{value} is not less than {self.lte}."
        return True
    

class SkyPathValidator(SkyValidatorMixin, EventDispatcher):
    """
    Validator for file paths.
    
    Attributes:
        local_path: True if the path should be on the current machine.
    """

    local_path = BooleanProperty(False)

    def validate(self, field, value):
        """
        Checks to see if the value is a valid path form.
                
        Returns:
            str: an error string if value is invalid
            True: If the value is valid.
        """

        if self.local_path:
            if not path.exists(value):
                return f"{value} was not found."
        
        return True
