from kivy.properties import NumericProperty, BooleanProperty

from abc import ABC, abstractmethod
from typing import Union
from re import fullmatch
import os

class SkyValidator(ABC):
    @abstractmethod
    def validate(self, form, field, value) -> Union[bool, str]:
        pass

class SkyRequiredValidator(SkyValidator):
    required = BooleanProperty(False)

    def validate(self, form, field, value) -> Union[bool, str]:
        if self.required:
            if value is not None:
                return True
            else:
                return f"{field} is required."

class SkyNetworkValidator(SkyValidator):
    port = BooleanProperty(False)
    ip = BooleanProperty(False)

    def validate(self, form, field, value) -> Union[bool, str]:
        if self.port:
            ret_val = self._port(form, field, value)
            if type(ret_val) == str:
                return ret_val
        if self.ip:
            ret_val = self._ip(form, field, value)
            if type(ret_val) == str:
                return ret_val
        return True

    def _port(self, form, field, value) -> Union[bool, str]:
        if not value.isnumeric():
            return f"{value} is not a port number."
        if int(value) < 0 or int(value) > 65535:
            return f"{value} is not a valid port number."
        return True

    def _ip(self, form, field, value) -> Union[bool, str]:
        if not fullmatch(r"[0-9]?[0-9]?[0-9]\.[0-9]?[0-9]?[0-9]\.[0-9]?[0-9]?[0-9].[0-9]?[0-9]?[0-9]", value):
            return f"{value} is not in ipv4 format."
        for i in value.split('.'):
            if int(i) < 0 or int(i) > 255:
                return f"{value} is not a valid ip address"
        return True
    
class SkyPathValidator(SkyValidator):
    dir_path = BooleanProperty(False)
    local_path = BooleanProperty(False)

    def validate(self, form, field, value) -> Union[bool, str]:
        if self.local_path:
            if not os.path.exists(value):
                return f"{value} was not found."
        return True
    
class SkyNumberValidator(SkyValidator):
    integer = BooleanProperty(False)
    gt = NumericProperty(None)
    gte = NumericProperty(None)
    lt = NumericProperty(None)
    lte = NumericProperty(None)

    def validate(self, form, field, value) -> Union[bool, str]:
        if self.integer:
            ret_val = self._integer(form, field, value)
            if type(ret_val) == str:
                return ret_val
        if self.gt is not None:
            ret_val = self._gt(form, field, value)
            if type(ret_val) == str:
                return ret_val
        if self.gte is not None:
            ret_val = self._gte(form, field, value)
            if type(ret_val) == str:
                return ret_val
        if self.lt is not None:
            ret_val = self._lt(form, field, value)
            if type(ret_val) == str:
                return ret_val
        if self.lte is not None:
            ret_val = self._lte(form, field, value)
            if type(ret_val) == str:
                return ret_val
        return True
        
    def _integer(self, form, field, value) -> Union[bool, str]:
        if not value.isdigit():
            return f"{value} is not an integer."
        return True

    def _gt(self, form, field, value) -> Union[bool, str]:
        if self._integer and int(value) <= self.gt:
            return f"{value} is not greater than {self.gt}."
        return True

    def _gte(self, form, field, value) -> Union[bool, str]:
        if self._integer and int(value) < self.gte:
            return f"{value} is not greater than or equal to {self.gte}."
        return True

    def _lt(self, form, field, value) -> Union[bool, str]:
        if self._integer and int(value) >= self.lt:
            return f"{value} is not less than {self.lt}."
        return True
    
    def _lte(self, form, field, value) -> Union[bool, str]:
        if self._integer and int(value) > self.lte:
            return f"{value} is not less than {self.lte}."
        return True