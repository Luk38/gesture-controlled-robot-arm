import socket
import json

#Default values for the IP, PORT, and BUFFER_SIZE
IP  = "127.0.0.1" #localhost
PORT = 4999
BUFFER_SIZE = 1024

def hand_position_stream(ip=IP, port=PORT, buffer_size=BUFFER_SIZE):
    """
    Generator function to receive hand positions from a UDP socket.
    
    Args:
        ip (str): IP address to bind the socket to.
        port (int): Port number to bind the socket to.
        buffer_size (int): Size of the buffer for receiving data.
        
    returns:
        dict: Parsed hand data received from the socket.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    while True:
        data, addr = sock.recvfrom(buffer_size)
        hand_data = json.loads(data.decode())
        yield hand_data