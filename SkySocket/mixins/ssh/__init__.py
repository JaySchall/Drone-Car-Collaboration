"""Provides backend control and access to SSH connections."""

from functools import partial
import subprocess
from threading import Thread

from fabric import Connection
from kivy.clock import Clock
from kivy.event import EventDispatcher
from kivy.properties import BooleanProperty


class SSHConnection(EventDispatcher):
    """
    Contains attributes for and maintains SSH connections.
    
    Attributes:
        connected: True if active connection exists.
        online: True if IP is available to be connected to.
        hostname: The IP of the devices to connect to.
        username: The name of the user profile to connect to.
        password: The password for the user.
        open_sessions: A list of the active tmux sessions.
        c: The connection object from fabric.
    """

    connected = BooleanProperty(False)
    online = BooleanProperty(False)

    def __init__(self, h, n, p):
        """Assigns properties, creates Connection object, and creates loops."""

        self.hostname = h
        self.username = n
        self.password = p
        self.open_sessions = []
        self.c = Connection(host=self.hostname, user=self.username, 
                            connect_kwargs={"password": self.password})
        
        Clock.schedule_interval(self._update_connection, 5)
        Clock.schedule_interval(self.on_ping, 15)

    def cmd(self, com):
        """
        Runs the given command on the remote host.
        
        Args:
            com: The character for character command run on the remote host.
        """
        if self.c.is_connected:
            self.c.run(com, warn=True)
    
    def cd_cmd(self, com, dest):
        """
        Runs a command in a different directory on the remote host.
        
        Args:
            com: The character for character command run on the remote host.
            dest: The path to the desired directory to run the command.
        """
        if self.c.is_connected:
            with self.c.cd(dest):
                self.c.run(com, warn=True)

    def create_session(self, sname):
        """
        Creates a tmux session with the given, unique name.
        
        Args:
            sname: The name of the tmux session to be created.
        """
    
        if sname in self.open_sessions:
            print("This session already exists")
            return
        if self.c.is_connected:        
            self.open_sessions.append(sname)
            self.c.run(f"tmux new -d -s {sname}", warn=True)    
    
    def send_keys(self, sname, cmd):
        """
        Sends a command to a session and hits enter.
        
        Args:
            sname: The name of the session.
            cmd: The key for key command to be run.
        """

        if sname not in self.open_sessions:
            print("This session does not exist")
        self.c.run(f"tmux send-keys -t {sname} \"{cmd}\" ENTER" )

    def kill_session(self, sname):
        """
        Stops the tmux session of the given name.
        
        Args:
            sname: The name of the tmux session that will be terminated.
        """

        if sname not in self.open_sessions:
            print("This session isnt open")
            return
        
        if self.c.is_connected:
            self.open_sessions.remove(sname)
            self.c.run(f"tmux kill-session -t {sname}", warn=True)

    def create_task(self, sname, cmd):
        """
        Creates a session to run a given command or process.
        
        Args:
            sname: The name of the tmux session.
            cmd: The command to be run in the tmux session.
        """

        if sname in self.open_sessions:
            print("This session already exists")
            return
        if self.c.is_connected:
            self.open_sessions.append(sname)
            self.c.run(f"tmux new -d -s {sname} '{cmd}'", warn=True)

    def get_tmux_sessions(self):
        """Retrieves active tmux sessions on the remote host."""

        if self.c.is_connected:
            return self.c.run(f"tmux ls", warn=True)
        else:
            return ""

    def is_connected(self):
        """Returns the status of the connection object."""

        return self.c.is_connected
    
    def is_running(self, sname):
        """
        Gets if a tmux session of the given name is running on the remote host.
        
        Args:
            sname: The name of the session to be checked.
        Returns:
            bool: If the session is running on the remote host or not.
        """
        returned = self.get_tmux_sessions()
        if sname in str(returned):
            return True
        
        return False
    
    def disconnect(self):
        """Ends the SSH connection."""

        if self.c.is_connected:
            self.c.close()
        else:
            print("Already disconnected")

    def reconnect(self):
        """Attempts to establish a connection in a thread."""

        t2 = Thread(target=self._reconnect)
        t2.start()

    def _reconnect(self):
        """A threaded attempt to establish a connection with the remote host."""

        if not self.online:
            print(f"{self.hostname} is not online.")
        elif self.c.is_connected:
            print(f"Already connected to {self.hostname}")
        else:
            try:
                self.c.open()
            except Exception as e:
                print(str(e))
            finally:
                Clock.schedule_once(self._update_connection)

    def _update_online(self, val, *args):
        """
        Updates the online attribute to the status of the SSH connection.
        
        Args:
            val: The current online status.
        """

        if self.online and not val:
            self.online = val
        elif not self.online and val:
            self.online = val

    def _ping(self):
        """Pings the given IP twice to test if online."""

        ping = subprocess.Popen(
            ["ping", "-n", "2", self.hostname],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        out, error = ping.communicate()
        if "(0% loss)" not in out:
            Clock.schedule_once(partial(self._update_online, False))
        else:
            Clock.schedule_once(partial(self._update_online, True))
    
    def on_ping(self, *args):
        """Creates a thread to ping the hostname IP in."""

        t1 = Thread(target=self._ping)
        t1.start()

    def _update_connection(self, *args):
        """Updates the connected attribute."""

        if self.connected and not self.c.is_connected:
            self.connected = False
        elif not self.connected and self.c.is_connected:
            self.connected = True
