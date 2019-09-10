#!/usr/local/bin/python

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse
import sys
import pickle
from multiprocessing import Queue, Process, Value
import socket
import os
import math
import copy

# Global Variables and Flags
vehicle = None
ignore_target = True
tangential_speed = 50 # cm/s
circle_period = sys.maxint
home_location = None
last_centering_time = 0

# Process shared flags
identified = None
shutdown = None

# Sockets
image_socket = None
gps_socket = None
shell_socket = None

# Shared queues
image_data = Queue(maxsize=1)
gps_coordinates = Queue()
shell_commands = Queue()
last_image_location = Queue(maxsize=1)

# Simulator flag
SIM = False

def setup():
    global vehicle

    # Connect to the Vehicle
    print "Connecting to the vehicle..."
    if SIM == False:
        vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)
    else: 
        vehicle = connect('tcp:localhost:5760', baud=57600, wait_ready=True)

    # Initialize the vehicle
    while not vehicle.is_armable:
        print "Waiting for vehicle to initialize..."
        time.sleep(1)

    # Arm the vehicle
    # arm()

    print "Set default/target airspeed to 3"
    vehicle.airspeed = 3


def arm(): 
    global vehicle

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)


def takeoff(atargetaltitude=10):
    global vehicle, home_location
    home_location = copy.deepcopy(vehicle.location.global_frame)
    vehicle.home_location=vehicle.location.global_frame

    print "arming"
    # Arm the UAV
    arm()

    vehicle.mode = VehicleMode("GUIDED")

    print "Taking off!"
    vehicle.simple_takeoff(atargetaltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= atargetaltitude * 0.95:
            print "Reached target altitude"
            break
        time.sleep(1)



def return_to_launch():
    global vehicle

    print "Returning to Launch"
    vehicle.mode = VehicleMode("RTL")

    # Wait until the vehicle lands to process the next command
    # while vehicle.location.global_relative_frame.alt >= 0:
    #   time.sleep(1)


def end_flight():
    global vehicle, shutdown

    # Disarm the vehicle
    vehicle.armed = False
    while vehicle.armed == True:      
        print " Waiting for disarming..."
        time.sleep(1)

    # Close vehicle object before exiting script
    print "Close vehicle object"
    vehicle.close()

    # Shutdown the other processes
    with shutdown.get_lock():
        shutdown.value = 1


def go_to_coordinate(latitude, longitude, altitude=6, speed=1):
    global vehicle

    print "Navigating to point"
    vehicle.mode = VehicleMode("GUIDED")
    print latitude
    print longitude
    print type(latitude)
    print type(longitude)
    point = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(point, groundspeed=speed)

    # sleep so we can see the change in map
    # time.sleep(30)


def circle_POI():
    global vehicle, ignore_target, tangential_speed, circle_period
    
    # Serach for the target
    ignore_target = False

    # The circle radius in cm. Max 10000
    # The tangential speed is 50 cm/s
    speed = tangential_speed

    # Radius has to be increments of 100 cm and rate has to be in increments of 1 degree
    radius = int(100)
    period = 2*math.pi*radius / speed
    rate = int(360.0/period)

    vehicle.parameters["CIRCLE_RADIUS"] = radius
    vehicle.parameters["CIRCLE_RATE"] = rate

    vehicle.mode = VehicleMode("CIRCLE")

    # Update the global variable for the next circle
    circle_period = period


def update_circle_params():
    global vehicle, tangential_speed, circle_period

    current_radius = vehicle.parameters["CIRCLE_RADIUS"]
    current_rate = vehicle.parameters["CIRCLE_RATE"]

    new_radius = current_radius + 100
    new_period = 2*math.pi*new_radius / tangential_speed
    new_rate = int(360.0/new_period)

    vehicle.parameters["CIRCLE_RADIUS"] = new_radius
    vehicle.parameters["CIRCLE_RATE"] = new_rate

    # Update the global variable for the next circle
    circle_period = new_period


def stop():
    global vehicle

    vehicle.mode = VehicleMode("GUIDED")
    print "stopped"

def clearGPSQueue():
    global gps_coordinates

    # Deletes and reinstatiates the GPS queue
    del gps_coordinates
    gps_coordinates = Queue()

def check_user_control():
    global vehicle, SIM

    if SIM == True:
        return False

    value = vehicle.channels['8']

    # print value

    if value < 1450:
        return True
    else:
        return False

def printData():
    global vehicle, gps_coordinates, home_location

    print "Alt: ", vehicle.location.global_relative_frame.alt
    print "Lat: ", vehicle.location.global_relative_frame.lat
    print "Lon: ", vehicle.location.global_relative_frame.lon
    print "Mode: ", vehicle.mode
    print "Ignore: ", ignore_target
    print "Distance from home: ", get_distance_metres(home_location, vehicle.location.global_frame)
    print "N E D: ", vehicle.location.local_frame.north, vehicle.location.local_frame.east, vehicle.location.local_frame.down
    print "yaw: ", vehicle.attitude.yaw

def drop():
    os.system('python ../experiments/drop_gpio.py')

# http://python.dronekit.io/guide/copter/guided_mode.html
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_distance_linear(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return (math.sqrt(dlat*dlat) * 1.113195e5, math.sqrt(dlong*dlong) * 1.113195e5)

def condition_yaw(heading, relative=False):
    global vehicle

    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def goto_position_target_local_ned(north, east, down):
    global vehicle

    if down >= -1:
        print "Bad altitude parameter!!"
        return

    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    print "Going to ", north, east, down

def differential_NED(north, east, down):
    global vehicle

    N = vehicle.location.local_frame.north
    E = vehicle.location.local_frame.east
    D = vehicle.location.local_frame.down
    print vehicle.location.global_relative_frame.lat
    print "NED: ", N, E, D
    print north, east, down

    goto_position_target_local_ned(N + north, E + east, D + down)

# Function to translate the UAV some amount forward, to the right, and down
def differential_FRD(front, right, down):
    global vehicle

    yaw = vehicle.attitude.yaw #radians
    E = right * math.cos(yaw) - front * math.sin(yaw)
    N = right * math.sin(yaw) + front * math.cos(yaw)

    differential_NED(N, E, down)

def center():
    global last_image_location, last_centering_time

    # Make sure this routine doesnt get called more than once every five seconds.
    if time.time() - last_centering_time < 5:
        return False
    else:
        last_centering_time = time.time()

    print "Centering..."

    alt = vehicle.location.global_relative_frame.alt
    FOV = 48.1 # Degrees
    angle_345 = 36.87 # degrees

    # Calculate the actual viewing X,Y distances
    X = 2 * alt * math.tan(math.radians(FOV/2)) * math.cos(math.radians(angle_345))
    Y = 2 * alt * math.tan(math.radians(FOV/2)) * math.sin(math.radians(angle_345))

    try:
        (cx, cy) = last_image_location.get_nowait()
        print cx, cy
    except:
        print "No image data."
        return False

    # Calculate the actual distance between the drone the the target
    # Scale the pixel location to the real location
    right = (cx - 320) * X / 640
    front = (-cy + 240) * Y / 480
    print "Center (FRD): ", (front, right)

    if (abs(front) <= 0.5) and (abs(right) <= 0.5):
        print "Centered!"
        return True
    else:
        differential_FRD(front, right, 0)
        return False

def shell_handler(command):
    global gps_coordinates, ignore_target, vehicle, shell_commands

    print "Command recieved: ", command

    if command == "takeoff":
        # Blocking
        takeoff(6)

    elif command == "land":
        # Non-blocking
        return_to_launch()

    elif command == "end":
        # Kills everything
        end_flight()

    elif command == "stop":
        # Non-blocking
        stop()

    elif command == "circle":
        # Non-blocking
        circle_POI()

    elif command == "goto":
        # Non-blocking? 

        # Make sure there is a location to go to
        location = []
        try:
            location = gps_coordinates.get_nowait()
        except:
            print "No available GPS coordinate"
            return

        print location
        go_to_coordinate(location[0], location[1], altitude=6, speed=1)

    elif command == "ignore":
        ignore_target = True

    elif command == "search":
        ignore_target = False

    elif command == "clearq":
        clearGPSQueue()

    elif command == "print":
        printData()

    elif command == "override":
        ignore_target = True
        vehicle.mode = VehicleMode("LOITER")
        time.sleep(1)

    elif command == "drop":
        drop()

    elif command.split()[0] == "pos":
        vehicle.mode = VehicleMode("GUIDED") 
        try:
            front = float(command.split()[1])
            right = float(command.split()[2])
            down = float(command.split()[3])
            differential_FRD(front, right, down)
        except:
            print "Poorly formatted."

    elif command.split()[0] == "yaw":
        try:
            angle = float(command.split()[1])
            condition_yaw(angle)
        except:
            print "Poorly formatted."

    elif command == "center":
        while center() == False:
            if check_user_control() == True:
                print "User override break"
                break
            else:
                try:
                    data = shell_commands.get_nowait()
                    shell_commands.put(data)
                    print "Shell break"
                    break
                except:
                    pass
        print "Done centering."

    else:
        print "Not a vaild command."


def process_image_data():
    global image_data, identified, shutdown, image_socket, last_image_location

    while True:
        try:
            client_socket, address = image_socket.accept()
            print "Image socket connected from ", address
            break
        except:
            pass

    while True:

        # If it is time to shutdown
        with shutdown.get_lock():
            if shutdown.value == 1:
                print "Shutting down."
                break

        # image_socket is non-blocking, so an exception might be raised if there is no data in the socket
        try:
            # Get the data
            data_string = client_socket.recv(512)

            # Clear the current data in the shared queue
            try:
                image_data.get(block=False)
            except:
                pass

            # Add the new data
            data = pickle.loads(data_string)
            image_data.put(data)

            # If the target has been seen, set the flag
            with identified.get_lock():
                if data != -1:
                    print "Saw something!"
                    print data
                    try:
                        last_image_location.get_nowait()
                    except:
                        pass
                    last_image_location.put(data)
                    identified.value = 1
                else:
                    identified.value = 0
        except:
            pass

        

def process_gps_data():
    global gps_coordinates, shutdown, gps_socket

    while True:
        try:
            client_socket, address = gps_socket.accept()
            print "GPS socket connected from ", address
            break
        except:
            pass

    while True:
        # gps_socket is non-blocking, so an exception might be raised if there is no data in the socket
        
        # If it is time to shut down
        with shutdown.get_lock():
            if shutdown.value == 1:
                print "Shutting down"
                break

        try:
            data_string = client_socket.recv(512)
            data = pickle.loads(data_string)
            print data
            gps_coordinates.put(data)
        except:
            continue

def process_shell_data():
    global shell_commands, shutdown, shell_socket

    # Connect the shell socket
    # Must connect to shell before the UAV will arm
    while True:
        try:
            client_socket, address = shell_socket.accept()
            print "Shell socket connected from ", address
            break
        except:
            pass

    while True:
    
        # If it is time to shut down
        with shutdown.get_lock():
            if shutdown.value == 1:
                print "Shutting down"
                break

        # shell_socket is non-blocking, so an exception might be raised if there is no data in the socket
        try:
            # Recieve data from shell socket
            data_string = client_socket.recv(512)
            data = pickle.loads(data_string)

            # Put it in the commands queue to be processed
            shell_commands.put(data)
            
        except:
            pass

def main():
    global image_socket, gps_socket, shell_socket, vehicle, identified, shutdown, ignore_target

    # Multi-core shared variables
    identified = Value('i', 0)
    shutdown = Value('i', 0)

    # Start the other scripts
    # os.system('python ../PiCamera/target_identification.py')
    # os.system('python ./shell.py')
    # os.system('python ../GoToHere/gotohere.py')

    # To open a terminal and run a command: 
    # os.system("gnome-terminal -e 'bash -c \"sudo apt-get update; exec bash\"'")

    # Socket for listening to the target_identification script
    image_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    image_socket.setblocking(0)
    image_socket.bind(("",5001))
    image_socket.listen(5)

    # Socket for listening to the GPS coordinate script
    gps_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    gps_socket.setblocking(0)
    gps_socket.bind(("",5002))
    gps_socket.listen(5)

    # Socket for listening to the user shell
    shell_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    shell_socket.setblocking(0)
    shell_socket.bind(("",5003))
    shell_socket.listen(5)

    # Image information handler
    ImageProcess = Process(target=process_image_data)
    ImageProcess.start()

    # GPS information handler
    GPSProcess = Process(target=process_gps_data)
    GPSProcess.start()

    # Shell information handler
    ShellProcess = Process(target=process_shell_data)
    ShellProcess.start()

    # Initialize and arm the vehicle
    setup()

    # Time variable
    last_time = time.time()

    # Main control loop
    while True:

        # See if there are any new commands queued up and act accordingly
        try:
            data = shell_commands.get_nowait()
            shell_handler(data)
        except:
            pass

        # Check if the circle needs to be expanded
        if SIM == False and vehicle.mode == "CIRCLE" and (time.time() - last_time > circle_period):
            update_circle_params()
            last_time = time.time()
            print "Expanding circle."

        # Check and see if the target has been found
        # Stop only if it has and you want to stop
        with identified.get_lock():
            if identified.value == 1 and ignore_target == False:
                print "Target Found!!"
                stop()
                ignore_target = True

        # If it is time to shut down
        with shutdown.get_lock():
            if shutdown.value == 1:
                break

    # Wait for the child processes to terminate
    GPSProcess.join()
    print "GPS process shut down"
    ImageProcess.join()
    print "Image process shut down"
    ShellProcess.join()
    print "Shell process shut down"

    # Shutdown the sockets
    image_socket.shutdown(socket.SHUT_RDWR)
    gps_socket.shutdown(socket.SHUT_RDWR)
    shell_socket.shutdown(socket.SHUT_RDWR)

    # Close the sockets
    image_socket.close()
    gps_socket.close()
    shell_socket.close()

    exit()
    

if __name__ == '__main__':
    main()

