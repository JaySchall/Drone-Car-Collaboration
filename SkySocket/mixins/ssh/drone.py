from mixins.ssh import sshConnection

class droneConnection(sshConnection):
  
    def __init__(self, h, n, p):
        super().__init__(h, n, p)
        
        
