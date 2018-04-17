#!/usr/bin/env python
'''
@author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de
@date February 20016
'''
import rospy
import serial
import time
import std_msgs

class EmergencyButton:
    def __init__(self, port= "/dev/xeno_emergency_button"):
        self.port= port
        self.baudrate= 500000
        self.ser= serial.Serial()
        self.publisher= rospy.Publisher("stop/emergency_button",
                                        std_msgs.msg.Bool,
                                        queue_size= 1)
    def connect(self):        
        self.ser= serial.Serial(self.port, baudrate= self.baudrate)

    def publish(self):
        if not self.ser.isOpen():
            self.connect()
        status= not self.ser.getCD()
        self.publisher.publish(status)

def main():
    rospy.init_node('xeno_interface')
    eb= EmergencyButton()
    eb.connect() # Handle connection error
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        eb.publish()
        r.sleep()
    
if __name__ == '__main__':
    main()
    
