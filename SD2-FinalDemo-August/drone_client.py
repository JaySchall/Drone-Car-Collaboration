import logging
from socket import *
import time

SERVER_NAME = "192.168.11.133"  # Server IP (User-defined)
SERVER_PORT = 10600             # Server Port (Predefined)
CLIENT_SOCKET = socket(AF_INET, SOCK_STREAM)  # Client Socket Creation 
# Client Socket Creation (for second argument: SOCK_DGRAM=UDP, SOCK_STREAM=TCP)

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
    logging.info("Sending message: %s", var)
    try:
        CLIENT_SOCKET.send(str(var).encode())  # Convert var to string and then encode it
        logging.info("Message sent successfully!")
    except Exception as e:
        logging.error("Error sending message: %s", str(e))
        
# QUIT Protocol
def close_socket():
    CLIENT_SOCKET.close()
    logging.info("Socket closed.")

def establish_socket_connection():
    try:
        CLIENT_SOCKET.connect((SERVER_NAME, SERVER_PORT))  # Establish connection when the program begins
        logging.info("Connected to server: %s on port: %s", SERVER_NAME, SERVER_PORT)
    except ConnectionRefusedError as e:
        logging.error("Error connecting to server: %s", str(e))
        close_socket()
    # Debug messages to indicate the connection status
    if CLIENT_SOCKET.fileno() != -1:
        logging.info("Socket connection is active (socket file descriptor = %s).", CLIENT_SOCKET.fileno())
        return True
    else:
        logging.info("Socket connection is closed (socket file descriptor = %s).", CLIENT_SOCKET.fileno())
        return False
