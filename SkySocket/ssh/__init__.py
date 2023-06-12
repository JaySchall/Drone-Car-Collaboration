from fabric import Connection
DRONE_HOSTNAME = '192.168.11.1'
DRONE_USER = 'pi'             # Hostname
DRONE_SUDO_PASS = 'raspberry'        # sudo password
CAR_HOSTNAME= '192.168.11.133'
CAR_USER = 'pi'
CAR_SUDO_PASS = 'raspberry'

DRONE_DIR = 'python_files/sd1_demo'
DRONE_FILE = 'RedDetect_drone-SD1_demo-v4.py'
CAR_DIR = 'python_files/sd1_demo'
CAR_FILE = 'car_server.py'

SHITTY_NUMBER = 0

def connect():
    c = Connection(host=DRONE_HOSTNAME, user=DRONE_USER, connect_kwargs = {"password": DRONE_SUDO_PASS})
    c.run('pwd')
    c.close()

def SHITTY_BUTTON():
    global SHITTY_NUMBER
    if SHITTY_NUMBER%2 == 0:
        CAR.init(CAR_HOSTNAME, CAR_USER, CAR_SUDO_PASS)
        CAR.start_experiment(CAR_DIR, CAR_FILE)
        print('CAR THING RUN')
        DRONE.init(DRONE_HOSTNAME, DRONE_USER, DRONE_SUDO_PASS)
        DRONE.start_experiment(DRONE_DIR, DRONE_FILE)
        print('DRONE THING RUN')
        SHITTY_NUMBER = SHITTY_NUMBER+1
    else:    
        CAR.end()
        DRONE.end()
        SHITTY_NUMBER =SHITTY_NUMBER+1

def car_init():
    CAR.init(CAR_HOSTNAME, CAR_USER, CAR_SUDO_PASS)
    print("CAR ESTABLISHED")

def car_begin():
    try:
        CAR.start_experiment(CAR_DIR, CAR_FILE)
        print("CAR READY")
    except:
        print("CAR FAILED")

def drone_init():
    DRONE.init(DRONE_HOSTNAME, DRONE_USER, DRONE_SUDO_PASS)
    print("DRONE ESTABLISHED")

def drone_begin():
    try:
        DRONE.start_experiment(DRONE_DIR, DRONE_FILE)
        print("DRONE READY")
    except:
        print("DRONE FAILED")

def stop():
    CAR.end()
    DRONE.end()
    print("EXPERIMENT DONE")
    

class SSH():
    def init(self, hostname, user, password):
        self._hostname = hostname
        self._user = user
        self._password = password
        self.con = Connection(host=self._hostname, user=self._user, connect_kwargs = {"password": self._password})
    
    def start_experiment(self, dir_path, file_path):
        self.file = file_path
        self.con.run(f'python3 {dir_path}/{file_path}')

    def end(self):
        self.con.run(f'killall {self.file}')
        self.con.close()


CAR = SSH()
DRONE = SSH()

if __name__ == '__main__':
    c = SSH()
    c.init(CAR_HOSTNAME, CAR_USER, CAR_SUDO_PASS)
    c.start_experiment(CAR_DIR, CAR_FILE)
    input()
    c.end()