from socket import *
from picarx import Picarx
import time

SERVER_NAME = "192.168.11.133"                              # Server IP (car)
SERVER_PORT = 10600                                         # Server Port(Predefined)
SPEED = 0                                                   # Global speed variable
DEFAULT_SPEED = 0                                           # Default cruising speed
px = Picarx()

def warning(var):
    global SPEED
    if var == 2:
        if SPEED != 0:                                      # Slow car down by half
            SPEED = SPEED / 2
            print("Slowing down car. New speed:", SPEED)
            px.forward(SPEED)
    elif var == 3:
        raise NotImplementedError("Left turn not implemented yet")  # Turn car left
    elif var == 4:
        raise NotImplementedError("Right turn not implemented yet")  # Turn car right
    
def obstruction():
    global SPEED
    SPEED = 0
    print("Car stopped due to obstruction")
    px.forward(SPEED)                                       # Stop car

def continueDriving():
    global SPEED
    global DEFAULT_SPEED
    SPEED = DEFAULT_SPEED                                   # Restart car after stopping or slowing down
    print("Continuing driving. Speed set to:", SPEED)
    px.forward(SPEED)


def main():
    SERVER_SOCKET = socket(AF_INET, SOCK_STREAM)                # Server socket creation
    SERVER_SOCKET.bind((SERVER_NAME, SERVER_PORT))
    SERVER_SOCKET.listen(1)                                    # Maximum number of queued connections

    print('Car now listening on ' + SERVER_NAME + ':' + str(SERVER_PORT))

    while True:
        try:
            print("Waiting for incoming client connections...")
            CONNECTION_SOCKET, addr = SERVER_SOCKET.accept()     # TCP Connection Created
            print("Connection established with:", addr)

            with CONNECTION_SOCKET:
                while True:
                    PACKET = CONNECTION_SOCKET.recv(1024).decode()   # Receives command
                    print("Received packet:", PACKET)
                    if not PACKET:
                        obstruction()                              # Make a call to stop car
                    elif int(PACKET) == 1:
                        continueDriving()                          # Go back to normal
                    else:
                        warning(int(PACKET))                       # Adjust speed or direction
        except Exception as e:
            print("Error occurred during client connection:", str(e))

if __name__ == "__main__":
    main()
