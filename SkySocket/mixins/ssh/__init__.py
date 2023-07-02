from fabric import Connection
from kivy.clock import Clock
from kivy.event import EventDispatcher
from kivy.properties import BooleanProperty

import subprocess
from threading import Thread
from functools import partial

class sshConnection(EventDispatcher):
    connected = BooleanProperty(False)
    online = BooleanProperty(False)


    def __init__(self, h, n, p):
        self.hostname = h
        self.username = n
        self.password = p
        self.open_sessions = []
        self.c = Connection(host=self.hostname, user=self.username, connect_kwargs={"password": self.password})
        Clock.schedule_interval(self._update_connection, 5)
        Clock.schedule_interval(self.on_ping, 15)

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
        self.c.run(f"tmux new -d -s {sname}", warn=True)

    def kill_session(self, sname):
        if sname not in self.open_sessions:
            print("This session isnt open")
            return
        self.open_sessions.remove(sname)
        self.c.run(f"tmux kill-session -t {sname}", warn=True)
    
    def cmd_to_window(self, sname, cmd):
        # Not implemented in final build
        if sname not in self.open_sessions:
            print("This session isnt open")
            return
        self.c.run(f"tmux send-keys -t {sname} '{cmd}' Enter", warn=True)

    def create_task(self, sname, cmd):
        if sname in self.open_sessions:
            print("This session already exists")
            return
        self.open_sessions.append(sname)
        self.c.run(f"tmux new -d -s {sname} '{cmd}'", warn=True)

    def get_tmux_sessions(self):
        return self.c.run(f"tmux ls")

    def is_connected(self):
        return self.c.is_connected
    
    def is_running(self):
        return False
    
    def disconnect(self):
        if self.is_connected():
            self.c.close()
        else:
            print("Already disconnected")

    def reconnect(self):
        t2 = Thread(target=self._reconnect)
        t2.start()

    def _reconnect(self):
        if not self.online:
            print(f"{self.hostname} is not online.")
        elif self.is_connected():
            print(f"Already connected to {self.hostname}")
        else:
            try:
                self.c.open()
            except Exception as e:
                print(str(e))
            finally:
                Clock.schedule_once(self._update_connection)

    def _update_online(self, val, *args):
        if self.online and not val:
            self.online = val
        elif not self.online and val:
            self.online = val

    def _ping(self):
        ping = subprocess.Popen(
            ["ping", "-n", "2", self.hostname],
            stdout = subprocess.PIPE,
            stderr = subprocess.PIPE,
            text=True
        )
        out, error = ping.communicate()
        if "(0% loss)" not in out:
            Clock.schedule_once(partial(self._update_online, False))
        else:
            Clock.schedule_once(partial(self._update_online, True))
    
    def on_ping(self, *args):
        t1 = Thread(target=self._ping)
        t1.start()

    def _update_connection(self, *args):
        if self.connected and not self.c.is_connected:
            self.connected = False
        elif not self.connected and self.c.is_connected:
            self.connected = True
