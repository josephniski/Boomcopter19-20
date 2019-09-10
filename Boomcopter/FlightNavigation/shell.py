#!/usr/local/bin/python

import socket, pickle


def main():

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    while True:
        try:
            client_socket.connect(('localhost', 5003))
            break
        except:
            pass

    while True:

        print "\n---------------------------------------------------------------------------\n"
        print "Options: takeoff, land, end, stop, circle, goto, ignore, search, clearq, print, override, drop, pos (F R D), yaw (angle), center"

        command = raw_input("What shall I do next?\n")
        data = pickle.dumps(command)
        client_socket.send(data)



if __name__ == '__main__':
    main()
