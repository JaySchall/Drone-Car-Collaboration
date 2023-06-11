from fabric import Connection

DRONE_HOSTNAME = ''         # IP
DRONE_USER = ''             # Hostname
DRONE_SUDO_PASS = ''        # sudo password
CAR_HOSTNAME = ''
CAR_USER = ''
CAR_SUDO_PASS = ''

SERVER_DIR = '/home/pi/Python_files/SocketPrograms'
SERVER_FILE = 'server.py'
CLIENT_DIR = '/home/pi/'
CLIENT_FILE = 'RedDetect.py'

def connect():
    c = Connection(host=DRONE_HOSTNAME, user=DRONE_USER, connect_kwargs = {"password": DRONE_SUDO_PASS})
    c.run('pwd')
    c.close()

def start_experiment():
    car_con = Connection(host=CAR_HOSTNAME, user=CAR_USER, connect_kwargs = {"password": CAR_SUDO_PASS})
    car_con.run(f'cd {SERVER_DIR}')
    car_con.run(f'python3 {SERVER_FILE}')
    car_con.close()
    drone_con = Connection(host=DRONE_HOSTNAME, user=DRONE_USER, connect_kwargs = {"password": DRONE_SUDO_PASS})
    drone_con.run(f'cd {CLIENT_DIR}')
    drone_con.run(f'python3 {CLIENT_FILE}')
    drone_con.close()