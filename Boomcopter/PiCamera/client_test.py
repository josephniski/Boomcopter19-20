#!/usr/local/bin/python

import socket
import pickle
import time

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('localhost', 5001))

data = (500, 500)
data_string = pickle.dumps(data)

while True:
    client_socket.send(data_string)
    time.sleep(1)
