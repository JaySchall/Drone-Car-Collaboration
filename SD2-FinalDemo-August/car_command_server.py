import logging
from socket import *
from picarx import Picarx
import time

SERVER_NAME = "192.168.11.133"  # Server IP (car)
SERVER_PORT = 10600  # Server Port(Predefined)
SPEED = 3  # Global speed variable
DEFAULT_SPEED = 1  # Default cruising speed
STOP = 0  # Stop Car command
CONT_DRIVE = 1  # Continue driving after a Stop Car command
REDUCE_SPEED = 2  # Reduce speed by some factor command (currently set to 1/2)
TURN_LEFT = 3  # Turn Left command
TURN_RIGHT = 4  # Turn Right command
ALL_CLEAR = 5  # No action necessary (essentially a null command)
px = Picarx()

# Configure logging to write to a log file and console
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("car_log.txt", mode="w"),
        logging.StreamHandler()
    ]
)

def warning(var):
    global SPEED
    if var == REDUCE_SPEED:
        if SPEED != 0:  # Slow car down by half
            SPEED = SPEED / 2
            logging.info("Slowing down car. New speed: %s", SPEED)
            px.forward(SPEED)
    elif var == TURN_LEFT:
        raise NotImplementedError("Left turn not implemented yet")  # Turn car left
    elif var == TURN_RIGHT:
        raise NotImplementedError("Right turn not implemented yet")  # Turn car right

def obstruction():
    global SPEED
    SPEED = STOP
    logging.info("Car stopped due to obstruction")
    px.forward(SPEED)  # Stop car

def continueDriving():
    global SPEED
    global DEFAULT_SPEED
    SPEED = DEFAULT_SPEED  # Restart car after stopping or slowing down
    logging.info("Continuing driving. Speed set to: %s", SPEED)
    px.forward(SPEED)

def main():
    SERVER_SOCKET = socket(AF_INET, SOCK_STREAM)  # Server socket creation
    SERVER_SOCKET.bind((SERVER_NAME, SERVER_PORT))
    SERVER_SOCKET.listen(1)  # Maximum number of queued connections

    logging.info('Car now listening on %s:%s', SERVER_NAME, SERVER_PORT)

    connection_established = False  # Flag to track if a connection is established
    timer_duration = 10  # Number of seconds to wait for a connection
    timer_start = time.time()  # Start the timer

    while not connection_established and (time.time() - timer_start) < timer_duration:
        try:
            logging.info("Waiting for incoming client connections...")
            CONNECTION_SOCKET, addr = SERVER_SOCKET.accept()  # TCP Connection Created
            logging.info("Connection established with: %s", addr)
            logging.info('Car now driving at %s...', SPEED)
            px.forward(SPEED)
            connection_established = True
        except Exception as e:
            logging.error("Error occurred during client connection: %s", str(e))

    if not connection_established:
        logging.info("No incoming connections. Exiting the program.")
        return

    stop_command_received = False
    stop_timer_start = None
    stop_timer_duration = 5  # Number of seconds to drop packets after receiving a stop command

    while True:
        
        if stop_command_received:
            elapsed_time = time.time() - stop_timer_start
            time_left = stop_timer_duration - elapsed_time
            if elapsed_time < stop_timer_duration:
                logging.info("STOP command received; now stopping and ignoring drone messages for %s seconds; Time left: %s", stop_timer_duration, time_left)
            else:
                stop_command_received = False
                stop_timer_start = None
                continueDriving() # Continue to normal driving
        try:
            PACKET = CONNECTION_SOCKET.recv(1).decode()  # Receives command (buffer size set to 1 byte --> recv(1))
            
            # The subsequent packet reception will be received but not processed UNLESS it is another STOP command
            if stop_command_received:
                # if another STOP packet was received, then restart the timer and command car to stop again for safety
                if int(PACKET) == STOP:
                    stop_command_received = True
                    stop_timer_start = time.time()
                    obstruction()  # Make a call to stop car
                    logging.info("Received and PROCESSING packet [0=stop,1=cont_drive,2=red_speed,3=L, 4=R,5=clear]: %s", int(PACKET))

                else:
                    logging.info("Received AND IGNORED packet [0=stop,1=cont_drive,2=red_speed,3=L, 4=R,5=clear]: %s", int(PACKET))
                    continue #Continue to start of while loop and do not process the PACKET
            else:
                #else process PACKET:
                logging.info("Received and PROCESSING packet [0=stop,1=cont_drive,2=red_speed,3=L, 4=R,5=clear]: %s", int(PACKET))
                if int(PACKET) == STOP:
                    stop_command_received = True
                    stop_timer_start = time.time()
                    obstruction()  # Make a call to stop car
                elif int(PACKET) == ALL_CLEAR:
                    continue  # No actions necessary
                elif int(PACKET) == CONT_DRIVE:
                    continueDriving()  # Continue to normal driving
                else:
                    warning(int(PACKET))  # Adjust speed or direction

        except Exception as e:
            logging.error("Error occurred during client connection: %s", str(e))

if __name__ == "__main__":
    main()
