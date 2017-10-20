#!/usr/bin/python

from geometry_msgs.msg import Twist
import rospy
from time import time
from time import sleep
import threading
import wiringpi
import os

#os.system("gpio export 12 out")
#os.system("gpio export 13 out")
#os.system("gpio export 6 out")
#os.system("gpio export 16 out")

ENABLE_MOTORS = True
DEBUG = False

rospy.init_node('Teleop_listener', anonymous=True)

lastDataReceivedTime = time()
motorPower = 512.0

class motorDriver:
    def __init__(self, pwmPin, dirPin):
        self._PWM_OUTPUT = 2
        self._IN = 0
        self._OUT = 1
        self.pwmPin = pwmPin
        self.dirPin = dirPin
        wiringpi.pinMode(self.pwmPin, self._PWM_OUTPUT)
        wiringpi.pinMode(self.dirPin, self._OUT)

    def setPower(self, value):
    	setting = int(abs(value)*1023.0)
	#if DEBUG:
	#print "Setting motor: " + str(setting)
    	if value<0:
    		wiringpi.digitalWrite(self.dirPin, 1)
    	else:
    		wiringpi.digitalWrite(self.dirPin, 0)

    	wiringpi.pwmWrite(self.pwmPin, setting)

    def deinit(self):
        wiringpi.pinMode(self.pwmPin, self._IN)
        wiringpi.pinMode(self.dirPin, self._IN)

def rescale(input):
    if input>0:
        return min((input+0.3, 1.0))
    elif input<0:
        return max((-1.0, input-0.3))
    else:
        return 0

def callback(data):
    global lastDataReceivedTime
    lastDataReceivedTime = time()
	#RPower = int((motorPower * (data.linear.x - data.angular.z))/MAXRANGE))
    RPower = rescale((data.linear.x + data.angular.z)/2.0)
    LPower = rescale((-data.linear.x + data.angular.z)/2.0)
    if DEBUG:
        rospy.loginfo('Direct: ' + str(RPower) + ' ' + str(LPower))
    if ENABLE_MOTORS:
        LeftMotors.setPower(LPower)
        RightMotors.setPower(RPower)

def watchdog():
    global lastDataReceivedTime
    while not rospy.is_shutdown():
        if (lastDataReceivedTime + 0.5) < time():
            if ENABLE_MOTORS:
                LeftMotors.setPower(0.0)
                RightMotors.setPower(0.0)
            if DEBUG:
		rospy.loginfo('Watchdog: 0 0 ')
        sleep(0.1)

result = wiringpi.wiringPiSetupGpio()
print result
RightMotors = motorDriver(12, 6)
LeftMotors = motorDriver(13, 16)

watchdog_thread = threading.Thread(target=watchdog)
watchdog_thread.start()
rospy.Subscriber("/chrzaszcz/cmd_vel", Twist, callback)
try:
    rospy.spin()
finally:
    LeftMotors.deinit()
    RightMotors.deinit()
    print "Shutting down..."
