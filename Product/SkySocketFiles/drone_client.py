import logging
import socket
import time

CONNECTED_TO_SERVER = False     # Track connection status of drone to the car
SERVER_NAME = "192.168.11.133"  # Server IP (User-defined)
SERVER_PORT = 10600             # Server Port (Predefined)
CLIENT_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Client Socket Creation 
# Client Socket Creation (for second argument: SOCK_DGRAM=UDP, SOCK_STREAM=TCP)

# Clear log file (by opening in write mode) before reopening in append mode
with open("red_and_edge_object_detection_log.txt", mode="w"):
    pass

# Create a logger instance for file logging
file_logger = logging.getLogger(__name__ + '--file_logger')
file_logger.setLevel(logging.INFO) # Write to log file with INFO and higher messages
file_formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("red_and_edge_object_detection_log.txt", mode="a")  # Open in append mode so file is not reset
file_handler.setFormatter(file_formatter)
file_logger.addHandler(file_handler)

# Create a logger instance for console logging
console_logger = logging.getLogger(__name__ + '--console_logger')
console_logger.setLevel(logging.INFO)  # Console logger set to capture WARNING and higher messages
console_formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
console_handler = logging.StreamHandler()
console_handler.setFormatter(console_formatter)
console_logger.addHandler(console_handler)


''' 
Send a message to the car using the following protocol:

0 = Stop due to obstruction
1 = Continue to normal driving
2 = Slow down due to possible obstruction
3 = Turn car left to avoid obstruction to the right
4 = Turn car right to avoid obstruction to the left
5 = All clear (no perceived threats or recovery actions necessary; no changes)
'''

def message_car(var):
    global CONNECTED_TO_SERVER
    
    file_logger.info("Sending message: %s", var)
    try:
        CLIENT_SOCKET.send(str(var).encode())  # Convert var to string and then encode it
        file_logger.info("Message sent successfully!")
        console_logger.info("Message sent successfully!")
        
    except Exception as e:
        file_logger.error("Error sending message: %s", str(e))
        console_logger.error("Error sending message: %s", str(e))
        close_socket() # close the socket so that future reestablishment can be attempted...
        CONNECTED_TO_SERVER = False
        
        
# QUIT Protocol
def close_socket():
    CLIENT_SOCKET.close()
    file_logger.info("Socket closed.")

def establish_socket_connection():
    global CONNECTED_TO_SERVER
    
    if not CONNECTED_TO_SERVER: # try to establish connection only if not already connected
        try:
            CLIENT_SOCKET.connect((SERVER_NAME, SERVER_PORT))  # Establish connection
            
            file_logger.info("Connected to server: %s on port: %s", SERVER_NAME, SERVER_PORT)
            print(f"Connected to server: {SERVER_NAME} on port: {SERVER_PORT}")
            
            file_logger.info("Socket connection is active (socket file descriptor = %s).", CLIENT_SOCKET.fileno())
            print(f"Socket connection is active (socket file descriptor = {CLIENT_SOCKET.fileno()}).")
            
            CONNECTED_TO_SERVER = True
            return True
        except socket.error as e:
            file_logger.error("Error connecting to server: %s", str(e))
            console_logger.error("Error connecting to server: %s", str(e))
            file_logger.info("Socket connection is closed (socket file descriptor = %s).", CLIENT_SOCKET.fileno())
            console_logger.info("Socket connection is closed (socket file descriptor = %s).", CLIENT_SOCKET.fileno())
            close_socket()
            return False
    else:
        return True # already connected to server; no action necessary
