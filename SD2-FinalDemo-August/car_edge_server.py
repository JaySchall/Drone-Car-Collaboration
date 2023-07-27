import logging
from socket import *

SERVER_NAME = "192.168.11.133"  # Server IP (User-defined)
SERVER_PORT = 10600             # Server Port (Predefined)
CLIENT_SOCKET = socket(AF_INET, SOCK_STREAM)  # Client Socket Creation 

# Create a logger instance
edge_server_logger = logging.getLogger(__name__)

# Clear log file before reopening in append mode
with open("yoloShapeDetect_log.txt", "w"):
    pass

# Configure logging to write to a log file and console
edge_server_logger.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - [%(name)s] - %(levelname)s - %(message)s")
file_handler = logging.FileHandler("yoloShapeDetect_log.txt")
file_handler.setFormatter(formatter)
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
edge_server_logger.addHandler(file_handler)
edge_server_logger.addHandler(stream_handler)

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
    edge_server_logger.info("Sending message: %s", var)
    try:
        CLIENT_SOCKET.send(str(var).encode())  # Convert var to string and then encode it
        edge_server_logger.info("Message sent successfully!")
    except Exception as e:
        edge_server_logger.error("Error sending message: %s", str(e))

# QUIT Protocol
def close_socket():
    CLIENT_SOCKET.close()
    edge_server_logger.info("Socket closed.")

def establish_socket_connection():
    try:
        CLIENT_SOCKET.connect((SERVER_NAME, SERVER_PORT))
        edge_server_logger.info("Connected to server: %s on port: %s", SERVER_NAME, SERVER_PORT)
    except ConnectionRefusedError as e:
        edge_server_logger.error("Error connecting to server: %s", str(e))
        close_socket()
    # Debug messages to indicate the connection status
    if CLIENT_SOCKET.fileno() != -1:
        edge_server_logger.info("Socket connection is active (socket file descriptor = %s).", CLIENT_SOCKET.fileno())
        return True
    else:
        edge_server_logger.info("Socket connection is closed (socket file descriptor = %s).", CLIENT_SOCKET.fileno())
        return False


# For testing purposes only:
if __name__ == "__main__":
    import time
    from datetime import datetime
    print("This program is not intended to be run as a main program, but it can be ran as a main program for testing purposes.")
    
    # Call the establish_socket_connection function
    if establish_socket_connection():
        while True:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")  # Get current timestamp
            print(f"Time Stamp: {timestamp} - This program is being ran as the main thread for testing purposes - CTRL + c to exit...")
            time.sleep(5)
    else:
        print("Failed to establish socket connection. Exiting...")
