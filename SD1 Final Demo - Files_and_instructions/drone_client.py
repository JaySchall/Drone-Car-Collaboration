from socket import *

SERVER_NAME = "192.168.11.133"                  # Server IP (User-defined)
SERVER_PORT = 10600                             # Server Port (Predefined)
CLIENT_SOCKET = socket(AF_INET, SOCK_STREAM)    # Client Socket Creation

# QUIT Protocol
def close_socket():
    CLIENT_SOCKET.close()
    print("Socket closed.")

''' 
Send a message to the car using following protocol:

0 = Stop due to obstruction
1 = Continue to normal driving
2 = Slow down due to possible obstruction
3 = Turn car left to avoid obstruction to right
4 = Turn car right to avoid obstruction to left
'''
def message_car(var):
    print("Sending message:", var)
    try:
        CLIENT_SOCKET.send(str(var).encode()) # Convert var to string and then encode it (encode expects a string that it will byte encode)
        print("Message sent successfully!")
    except Exception as e:
        print("Error sending message:", str(e))

try:
    CLIENT_SOCKET.connect((SERVER_NAME, SERVER_PORT))       # Establish connection when drone begins
    print("Connected to server:", SERVER_NAME, "on port:", SERVER_PORT)
except Exception as e:
    print("Error connecting to server:", str(e))

# Debug messages to indicate the connection status
if CLIENT_SOCKET.fileno() != -1:
    print("Socket connection is active.")
else:
    print("Socket connection is closed.")

