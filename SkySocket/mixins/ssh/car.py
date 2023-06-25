from res.config import SkyConfig
from mixins.ssh import sshConnection

class carConnection(sshConnection):
    
    def __init__(self, h, n, p):
        super().__init__(h, n, p)