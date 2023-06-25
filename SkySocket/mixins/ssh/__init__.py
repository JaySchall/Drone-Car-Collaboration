from fabric import Connection

class sshConnection:

    def __init__(self, h, n, p):
        self.hostname = h
        self.username = n
        self.password = p
        self.open_sessions = []
        self.c = Connection(host=self.hostname, user=self.username, connect_kwargs={"password": self.password})

    def cmd(self, com):
        self.c.run(com, warn=True)
    
    def cd_cmd(self, com, dest):
        with self.c.cd(dest):
            self.c.run(com, warn=True)

    def create_session(self, sname):
        if sname in self.open_sessions:
            print("This session already exists")
            return
        self.open_sessions.append(sname)
        self.c.run(f"tmux new -d -s {sname}")

    def kill_session(self, sname):
        if sname not in self.open_sessions:
            print("This session isnt open")
            return
        self.open_sessions.remove(sname)
        self.c.run(f"tmux kill-session -t {sname}")
    
    def cmd_to_window(self, sname, cmd):
        if sname not in self.open_sessions:
            print("This session isnt open")
            return
        self.c.run(f"tmux send-keys -t {sname} '{cmd}' Enter")

    def create_task(self, sname, cmd):
        if sname in self.open_sessions:
            print("This session already exists")
            return
        self.open_sessions.append(sname)
        self.c.run(f"tmux new -d -s {sname} '{cmd}'")

    def is_connected(self):
        return self.c.is_connected
    
    def disconnect(self):
        if self.is_connected():
            self.c.close()
        else:
            print("Already disconnected")

    def reconnect(self):
        if self.is_connected():
            print("Already connected")
        else:
            self.c = Connection(host=self.hostname, user=self.username, connect_kwargs={"password": self.password})
