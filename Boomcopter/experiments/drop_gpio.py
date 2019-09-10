#!/usr/bin/python
import time

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

time.sleep(0.5)

GPIO.output(32, GPIO.HIGH)
GPIO.output(31, GPIO.HIGH)
GPIO.output(35, GPIO.HIGH)
GPIO.output(38, GPIO.HIGH)

time.sleep(1)

GPIO.output(32, GPIO.LOW)
GPIO.output(31, GPIO.LOW)
GPIO.output(35, GPIO.LOW)
GPIO.output(38, GPIO.LOW)

GPIO.cleanup()
exit()

