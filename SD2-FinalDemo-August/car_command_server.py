import logging
import socket
from picarx import Picarx
import time
import threading

# Global variables
SERVER_NAME = "192.168.11.133"  # Server IP (car)
SERVER_PORT = 10600  # Server Port (Predefined)
SPEED = 1  # Global speed variable
DEFAULT_SPEED = 1  # Default cruising speed
NUM_CLIENTS_ALLOWED = 2 # Max number of clients allowed to connect to server
NUM_CLIENTS_REQUIRED = 2 # Max number of clients required to connect to server
NUM_CLIENTS_CONNECTED = 0 # for multithreading and recovery purposes - track num of connected clients
ALL_CLIENTS_CONNECTED = threading.Event() # use this to track if all client threads still have a server connection
CLIENTS_LOCK = threading.Lock() # use this to lock sections of code that access the num_clients_connected global variable
LAST_COMMAND_RECEIVED = None # track the last packet received
EXIT_PROGRAM = threading.Event() # use this to control exiting program, including terminating threads

#Car commands
STOP = 0  # Stop Car command
CONT_DRIVE = 1  # Continue driving after a Stop Car command
REDUCE_SPEED = 2  # Reduce speed by some factor command (currently set to 1/2)
TURN_LEFT = 3  # Turn Left command
TURN_RIGHT = 4  # Turn Right command
ALL_CLEAR = 5  # No action necessary (essentially a null command)

#create instance of pi car
px = Picarx()

# Create a logger instance for file logging
file_logger = logging.getLogger(__name__ + '--file_logger')
file_logger.setLevel(logging.INFO)
file_formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("car_command_server_log.txt", mode="w")
file_handler.setFormatter(file_formatter)
file_logger.addHandler(file_handler)

# Create a logger instance for console logging
console_logger = logging.getLogger(__name__ + '--console_logger')
console_logger.setLevel(logging.INFO)  # Console logger set to capture WARNING and higher messages
console_formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
console_handler = logging.StreamHandler()
console_handler.setFormatter(console_formatter)
console_logger.addHandler(console_handler)

def warning(command, client_addr):
    global SPEED
    if command == REDUCE_SPEED:
        if SPEED != 0:  # Slow car down by half
            SPEED = SPEED / 2
            file_logger.info("Client %s: Slowing down car. New speed: %s", client_addr, SPEED)
            console_logger.warning("Client %s: Slowing down car. New speed: %s", client_addr, SPEED)
            px.forward(SPEED)
    elif command == TURN_LEFT:
        raise NotImplementedError("Left turn not implemented yet")  # Turn car left
    elif command == TURN_RIGHT:
        raise NotImplementedError("Right turn not implemented yet")  # Turn car right
    
def connection_lost_stop_car(client_addr):
    global SPEED
    SPEED = STOP
    file_logger.warning("Car STOPPED due to closed or lost connection to client %s", client_addr)
    console_logger.warning("Car STOPPED due to closed or lost connection to client %s", client_addr)
    px.forward(SPEED)  # Stop car
    
def obstruction():
    global SPEED
    SPEED = STOP
    file_logger.info("Car stopped due to obstruction")
    console_logger.info("Car stopped due to obstruction")
    px.forward(SPEED)  # Stop car

def continueDriving():
    global SPEED
    global DEFAULT_SPEED
    SPEED = DEFAULT_SPEED  # Restart car after stopping or slowing down
    file_logger.info("Continuing driving. Speed set to: %s", SPEED)
    console_logger.info("Continuing driving. Speed set to: %s", SPEED)
    px.forward(SPEED)

def handle_client_connection_thread(connection_socket, client_addr):
    global SPEED, DEFAULT_SPEED, NUM_CLIENTS_CONNECTED
   
    try:
        while True: # run this loop until there is a connection error
            
            if EXIT_PROGRAM.is_set():
                break # exit this thread if exit program event is set
            
            # Wait until all clients are connected
            while not ALL_CLIENTS_CONNECTED.is_set() and not EXIT_PROGRAM.is_set():
                time.sleep(1)  # Wait until all clients are connected
                
            # Now receive packets while number of clients connected is adequate.
            while ALL_CLIENTS_CONNECTED.is_set() and not EXIT_PROGRAM.is_set():
                # Now, receive a packet:
                packet = connection_socket.recv(1).decode()  # Receives command from a client (buffer size set to 1 byte --> recv(1))
                file_logger.info("Received and PROCESSING packet from CLIENT: (%s), [0=stop,1=cont_drive,2=red_speed,3=L, 4=R,5=clear]: %s",
                                client_addr, int(packet))
                console_logger.info("Received and PROCESSING packet from CLIENT: (%s), [0=stop,1=cont_drive,2=red_speed,3=L, 4=R,5=clear]: %s",
                                client_addr, int(packet))
                if int(packet) == STOP:
                    obstruction()  # Make a call to stop the car
                elif int(packet) == ALL_CLEAR:
                    continue  # No actions necessary
                elif int(packet) == CONT_DRIVE:
                    continueDriving()  # Continue to normal driving
                else:
                    warning(int(packet), client_addr)  # Adjust speed or direction

    except socket.error as e:
        file_logger.error("Error occurred during client (%s) connection: %s", client_addr, str(e))
        console_logger.error("Error occurred during client (%s) connection: %s", client_addr, str(e))
        
    #Now, we have left the while loop due to error or not all sockets connected; need to decerement the number of clients connected:
    finally:
        # Ensure the lock is acquired to safely decrement the number of connected clients
        with CLIENTS_LOCK:
            NUM_CLIENTS_CONNECTED -= 1

            # Client is about to disconnect; clear the event so that car will stop and wait for all connections to be established
            if NUM_CLIENTS_CONNECTED < NUM_CLIENTS_REQUIRED:
                ALL_CLIENTS_CONNECTED.clear() # set to false
                connection_lost_stop_car(client_addr) # stop car

            # Close the client socket and perform cleanup
            connection_socket.close()
            file_logger.info("Connection closed with client (%s)", client_addr)
            print("Connection closed with client", client_addr)

def main():
    global NUM_CLIENTS_CONNECTED, EXIT_PROGRAM
    SERVER_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Server socket creation
    SERVER_SOCKET.bind((SERVER_NAME, SERVER_PORT))
    SERVER_SOCKET.listen(NUM_CLIENTS_ALLOWED)  # Maximum number of queued connections

    file_logger.info('Car now listening on %s:%s', SERVER_NAME, SERVER_PORT)
    print('Car now listening on %s:%s' % (SERVER_NAME, SERVER_PORT))

    connection_threads = [] # Thread IDs stored in this list

    while True:
        try:
            if not ALL_CLIENTS_CONNECTED.is_set():
                file_logger.info("Waiting for incoming client connections...(current number of established connections: %s)", NUM_CLIENTS_CONNECTED)
                print("Waiting for incoming client connections...(current number of established connections: %s)" % NUM_CLIENTS_CONNECTED)
            
                #This program will hang after calling SERVER_SOCKET.accept() until connection is requested and accepted:
                connection_socket, addr = SERVER_SOCKET.accept()  # TCP Connection Created - note that addr is a tuple of (IP,PORT_NUM)
                file_logger.info("Connection established with: %s:%s", addr[0], addr[1])
                print("Connection established with: %s:%s" % addr)
                
                # Each client thread will run the handle_client_connection thread, 
                # as this program will spawn a child thread for each client.
                client_thread = threading.Thread(target=handle_client_connection_thread, args=(connection_socket, addr))
                client_thread.setDaemon(True) # make daemon thread so when main exits, this thread will terminate and not make main wait.
                client_thread.start()
                connection_threads.append(client_thread)
                
                # Acquire the lock to safely increment the number of connected clients, then check if num client req is sufficient to start driving
                with CLIENTS_LOCK:
                    NUM_CLIENTS_CONNECTED += 1
                    if NUM_CLIENTS_CONNECTED == NUM_CLIENTS_REQUIRED:
                        ALL_CLIENTS_CONNECTED.set() # set this so that client threads can start sending messages.
                        SPEED = DEFAULT_SPEED # reset speed to default speed
                        file_logger.info('Now connected to drone and edge server - Car now driving at %s...', SPEED)
                        print('Now connected to drone and edge server - Car now driving at speed %s...' % SPEED)
                        px.forward(SPEED) #begin driving now that required number of client command nodes connected
            else:
                time.sleep(1)  # Wait for 1 second before checking for the next client connection
        except KeyboardInterrupt:
            print("\nCTRL + C detected from user input. Exiting the program...")
            file_logger.error("\nCTRL + C detected from user input. Exiting the program...")
            EXIT_PROGRAM.set() # trigger exit program event so all threads will terminate
            break
        except Exception as e:
            file_logger.error("Error occurred during client connection: %s", str(e))
            console_logger.error("Error occurred during client connection: %s", str(e))

    # close socket before exiting program
    SERVER_SOCKET.close() 
    time.sleep(5) # give OS time to release socket

    # Join all client socket connection threads before exiting program so that all threads are terminated first
    for thread in connection_threads:
        if not thread.isDaemon(): # we only need to join non-daemon threads, since daemon threads will exit and not make the main thread wait.
            thread.join()


if __name__ == "__main__":
    main()
