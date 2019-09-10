import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
#Solenoid 1
GPIO.setup(38, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)
#Solenoid 2
GPIO.setup(35, GPIO.OUT)
GPIO.setup(37, GPIO.OUT)
#Solenoid 3
GPIO.setup(31, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
#Solenoid 4
GPIO.setup(32, GPIO.OUT)
GPIO.setup(36, GPIO.OUT)

while True:
    print '\n'
    print 'Commands: close #, open #, exit'
    cmd = raw_input("Command: ")

    if cmd == 'close 1':
        GPIO.output(38, GPIO.LOW)
    elif cmd == 'open 1':
        GPIO.output(38, GPIO.HIGH)
    elif cmd == 'close 2':
        GPIO.output(35, GPIO.LOW)
    elif cmd == 'open 2':
        GPIO.output(35, GPIO.HIGH)
    elif cmd == 'close 3':
        GPIO.output(31, GPIO.LOW)
    elif cmd == 'open 3':
        GPIO.output(31, GPIO.HIGH)
    elif cmd == 'close 4':
        GPIO.output(32, GPIO.LOW)
    elif cmd == 'open 4':
        GPIO.output(32, GPIO.HIGH)
    elif cmd == 'close':
        GPIO.output(32, GPIO.LOW)
        GPIO.output(31, GPIO.LOW)
        GPIO.output(35, GPIO.LOW)
        GPIO.output(38, GPIO.LOW)
    elif cmd == 'open':
        GPIO.output(32, GPIO.HIGH)
        GPIO.output(31, GPIO.HIGH)
        GPIO.output(35, GPIO.HIGH)
        GPIO.output(38, GPIO.HIGH)
    elif cmd == 'exit':
        GPIO.cleanup()
        exit()

