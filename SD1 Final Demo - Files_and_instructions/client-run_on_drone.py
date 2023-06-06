from socket import *

SERVER_NAME = "192.198.1.133"                     # Server IP (User defiend)
SERVER_PORT = 10600                             # Server Port (Predefined)
CLIENT_SOCKET = socket(AF_INET, SOCK_STREAM)    # Client Socket Creation

# QUIT Protocol
def close_socket():
    CLIENT_SOCKET.close()

''' 
Send a message to the car using following protocol:

0 = Stop due to obstruction
1 = Continue to normal driving
2 = Slow down due to possible obstruction
3 = Turn car left to avoid obstruction to right
4 = Turn car right to avoid obstruction to left
'''
def message_car(var):
    CLIENT_SOCKET.send(var.encode())

CLIENT_SOCKET.connect((SERVER_NAME, SERVER_PORT))       # Establish connection when drone begins



    

