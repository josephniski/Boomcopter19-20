import socket, pickle


def main():
    image_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    gps_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    while True:
        try:
            gps_socket.connect(('localhost', 5002))
            break
        except:
            pass

    while True:
        try:
            image_socket.connect(('localhost', 5001))
            break
        except:
            pass

    while True:

        print "\n---------------------------------------------------------------------------\n"
        command = raw_input("What type of data?\t")

        if command == "gps":
            try:
                lat = float(raw_input("Latitude:\t"))
                lon = float(raw_input("Longitude:\t"))
            except:
                continue

            data = (lat, lon)

            serialized_data = pickle.dumps(data)
            gps_socket.send(serialized_data)

        elif command == "image":
            try:
                X = float(raw_input("X:\t"))
                Y = float(raw_input("Y:\t"))
            except:
                continue

            data = (X, Y)

            if X == -1.0:
                data = -1

            print data
            serialized_data = pickle.dumps(data)
            image_socket.send(serialized_data)

        else:
            print "Not valid data"


if __name__ == '__main__':
    main()
