import logging
from socket import *
from picarx import Picarx
import time
import threading

# Global variables
SERVER_NAME = "192.168.11.133"  # Server IP (car)
SERVER_PORT = 10600  # Server Port (Predefined)
SPEED = 3  # Global speed variable
DEFAULT_SPEED = 1  # Default cruising speed
STOP = 0  # Stop Car command
CONT_DRIVE = 1  # Continue driving after a Stop Car command
REDUCE_SPEED = 2  # Reduce speed by some factor command (currently set to 1/2)
TURN_LEFT = 3  # Turn Left command
TURN_RIGHT = 4  # Turn Right command
ALL_CLEAR = 5  # No action necessary (essentially a null command)
px = Picarx()

# Configure logging to write to a log file and console - open in write mode so the file is first cleared (contents deleted)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - Client %(client_addr)s - %(message)s",
    handlers=[
        logging.FileHandler("car_log.txt", mode="w"),
        logging.StreamHandler()
    ]
)

def warning(command, client_addr):
    global SPEED
    if command == REDUCE_SPEED:
        if SPEED != 0:  # Slow car down by half
            SPEED = SPEED / 2
            logging.info("Client %s: Slowing down car. New speed: %s", client_addr, SPEED)
            px.forward(SPEED)
    elif command == TURN_LEFT:
        raise NotImplementedError("Left turn not implemented yet")  # Turn car left
    elif command == TURN_RIGHT:
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

def handle_client_connection(connection_socket, client_addr):
    global SPEED, DEFAULT_SPEED

    stop_command_received = False
    stop_timer_start = None
    stop_timer_duration = 5  # Number of seconds to drop packets after receiving a stop command

    while True:
        if stop_command_received:
            elapsed_time = time.time() - stop_timer_start
            time_left = stop_timer_duration - elapsed_time
            if elapsed_time < stop_timer_duration:
                logging.info("STOP command received from CLIENT: (%s); now stopping and ignoring drone messages for %s seconds; Time left: %s",
                             client_addr, stop_timer_duration, time_left)
            else:
                stop_command_received = False
                stop_timer_start = None
                continueDriving()  # Continue to normal driving

        try:
            packet = connection_socket.recv(1).decode()  # Receives command from a client (buffer size set to 1 byte --> recv(1))

            if stop_command_received:
                if int(packet) == STOP:
                    stop_command_received = True
                    stop_timer_start = time.time()
                    obstruction()  # Make a call to stop the car
                    logging.info("Received and PROCESSING packet from CLIENT: (%s), [0=stop,1=cont_drive,2=red_speed,3=L, 4=R,5=clear]: %s",
                                 client_addr, int(packet))
                else:
                    logging.info("Received AND IGNORED packet from CLIENT: (%s), [0=stop,1=cont_drive,2=red_speed,3=L, 4=R,5=clear]: %s",
                                 client_addr, int(packet))
                    continue  # Continue to the start of the while loop and do not process the packet
            else:
                logging.info("Received and PROCESSING packet from CLIENT: (%s), [0=stop,1=cont_drive,2=red_speed,3=L, 4=R,5=clear]: %s",
                             client_addr, int(packet))
                if int(packet) == STOP:
                    stop_command_received = True
                    stop_timer_start = time.time()
                    obstruction()  # Make a call to stop the car
                elif int(packet) == ALL_CLEAR:
                    continue  # No actions necessary
                elif int(packet) == CONT_DRIVE:
                    continueDriving()  # Continue to normal driving
                else:
                    warning(int(packet), client_addr)  # Adjust speed or direction

        except Exception as e:
            logging.error("Error occurred during client (%s) connection: %s", client_addr, str(e))
            break

    connection_socket.close()
    logging.info("Connection closed with client (%s)", client_addr)

def main():
    SERVER_SOCKET = socket(AF_INET, SOCK_STREAM)  # Server socket creation
    SERVER_SOCKET.bind((SERVER_NAME, SERVER_PORT))
    SERVER_SOCKET.listen(5)  # Maximum number of queued connections

    logging.info('Car now listening on %s:%s', SERVER_NAME, SERVER_PORT)

    connected_clients = 0  # Counter for connected clients
    connection_threads = []

    while connected_clients < 2:
        try:
            logging.info("Waiting for incoming client connections...")
            connection_socket, addr = SERVER_SOCKET.accept()  # TCP Connection Created
            logging.info("Connection established with: %s", addr)
            logging.info('Car now driving at %s...', SPEED)
            px.forward(SPEED)

            # Each client thread will run the handle_client_connection thread, 
            # as this program will spawn a child thread for each client.
            client_thread = threading.Thread(target=handle_client_connection, args=(connection_socket, addr))
            client_thread.start()
            connection_threads.append(client_thread)

            connected_clients += 1  # Increment the counter for each connected client

        except Exception as e:
            logging.error("Error occurred during client connection: %s", str(e))

        if connected_clients == 2:
            break

        time.sleep(1)  # Wait for 1 second before checking for the next client connection

    # Join all client socket connection threads before exiting program so that all threads are terminated first
    for thread in connection_threads:
        thread.join()

    SERVER_SOCKET.close()

if __name__ == "__main__":
    main()
