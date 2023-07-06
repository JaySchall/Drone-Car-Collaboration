from socket import *
import time

SERVER_NAME = "192.168.11.133"  # Server IP (User-defined)
SERVER_PORT = 10600             # Server Port (Predefined)
CLIENT_SOCKET = socket(AF_INET, SOCK_DGRAM)  # Client Socket Creation (for second arguemnt: SOCK_DGRAM=UDP, SOCK_STREAM=TCP)



''' 
Send a message to the car using the following protocol:

0 = Stop due to obstruction
1 = Continue to normal driving
2 = Slow down due to possible obstruction
3 = Turn car left to avoid obstruction to the right
4 = Turn car right to avoid obstruction to the left
5 = All clear (no perceived threats or recovery actions necessary; no changes)
'''
def wait_timer(seconds):
    print(f"Waiting for {seconds} seconds...")
    time.sleep(seconds)

def message_car(var):
    print("Sending message:", var)
    try:
        CLIENT_SOCKET.send(str(var).encode())  # Convert var to string and then encode it
        print("Message sent successfully!")
    except Exception as e:
        print("Error sending message:", str(e))
        
# QUIT Protocol
def close_socket():
    CLIENT_SOCKET.close()
    print("Socket closed.")

def establish_socket_connection():
    try:
        CLIENT_SOCKET.connect((SERVER_NAME, SERVER_PORT))  # Establish connection when the program begins
        print("Connected to server:", SERVER_NAME, "on port:", SERVER_PORT)
    except ConnectionRefusedError as e:
        print("Error connecting to server:", str(e))
        close_socket()
    # Debug messages to indicate the connection status
    if CLIENT_SOCKET.fileno() != -1:
        print(f"Socket connection is active (socket file descriptor = {CLIENT_SOCKET.fileno()}).")
        return True
    else:
        print(f"Socket connection is closed (socket file descriptor = {CLIENT_SOCKET.fileno()}).")
        return False

