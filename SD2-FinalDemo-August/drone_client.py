import logging
from socket import *
import time

SERVER_NAME = "192.168.11.133"  # Server IP (User-defined)
SERVER_PORT = 10600             # Server Port (Predefined)
CLIENT_SOCKET = socket(AF_INET, SOCK_STREAM)  # Client Socket Creation 
# Client Socket Creation (for second argument: SOCK_DGRAM=UDP, SOCK_STREAM=TCP)

# Create a logger instance
drone_client_logger = logging.getLogger(__name__)

# Clear log file before reopening in append mode
with open("red_and_edge_object_detection_log.txt", "w"):
    pass

# Configure logging to write to a log file and console
drone_client_logger.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("red_and_edge_object_detection_log.txt")
file_handler.setFormatter(formatter)
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
drone_client_logger.addHandler(file_handler)
drone_client_logger.addHandler(stream_handler)

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
    drone_client_logger.info("Sending message: %s", var)
    try:
        CLIENT_SOCKET.send(str(var).encode())  # Convert var to string and then encode it
        drone_client_logger.info("Message sent successfully!")
    except Exception as e:
        drone_client_logger.error("Error sending message: %s", str(e))
        
# QUIT Protocol
def close_socket():
    CLIENT_SOCKET.close()
    drone_client_logger.info("Socket closed.")

def establish_socket_connection():
    try:
        CLIENT_SOCKET.connect((SERVER_NAME, SERVER_PORT))  # Establish connection when the program begins
        drone_client_logger.info("Connected to server: %s on port: %s", SERVER_NAME, SERVER_PORT)
    except ConnectionRefusedError as e:
        drone_client_logger.error("Error connecting to server: %s", str(e))
        close_socket()
    # Debug messages to indicate the connection status
    if CLIENT_SOCKET.fileno() != -1:
        drone_client_logger.info("Socket connection is active (socket file descriptor = %s).", CLIENT_SOCKET.fileno())
        return True
    else:
        drone_client_logger.info("Socket connection is closed (socket file descriptor = %s).", CLIENT_SOCKET.fileno())
        return False
