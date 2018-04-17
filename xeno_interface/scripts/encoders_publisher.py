#!/usr/bin/env python

'''
@author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
@date February 2016
'''

import serial
import rospy
import sys
import time
import datetime
from geometry_msgs.msg import Vector3Stamped

class XenoEncoderReader:
    def __init__(self, port= '/dev/xeno_rear_encoders', baudrate= 115200):
        self.port= port
        # Important. It seems that when using lower bitrate values ,e.g. 9600 the microcontroller board sends
        # the data in a different format.
        self.baudrate= baudrate
        self.ser= serial.Serial()
        self.packet= ''
        self.buffer= ''
        self.packet_time= None

    def connect(self):
        print self.port,'@',self.baudrate,':', 'opening connection...'
        self.ser= serial.Serial(self.port, baudrate= self.baudrate)
        print 'Done connecting'

    def consume_bytes(self, n):
        self.packet+= self.buffer[0:n]
        self.buffer= self.buffer[n:]

    def print_packet(self):
        print ' '.join(map(lambda v: v.encode('hex'),
                           self.packet))

    def interpret_tick_byte(self, b):
        v= ord(self.packet[b])
        return v if v < 127 else v - 256

    def get_start_byte(self):
        raise NotImplementedError

    def get_packet_size(self):
        raise NotImplementedError
    
    def handle_packet(self):
        raise NotImplementedError

    def handle_data(self):
#        print 'Data arrived'
        if len(self.packet) == self.get_packet_size():
            self.handle_packet()
            self.packet= ''
        # Search for starting byte
        elif len(self.packet) == 0:
            start_idx= self.buffer.find(self.get_start_byte())
            if start_idx != -1:
                self.packet_time= datetime.datetime.now()
                self.buffer= self.buffer[start_idx:]
                self.consume_bytes(self.get_packet_size())
            else:
                # Clean the buffer
                self.buffer= ''
        elif len(self.packet) != 0:
            self.consume_bytes(self.get_packet_size() - len(self.packet))
        
    def spin(self):
        try:
            self.buffer+= self.ser.read(self.get_packet_size())
            while len(self.buffer) >= self.get_packet_size():
                self.handle_data()
        except serial.serialutil.SerialException, e:
            print 'Serial port exception...trying to reconnect'
            try:
                self.connect()
            except serial.serialutil.SerialException, e:
                print 'Reconnection failed'
            
class XenoRearEncoderPublisher(XenoEncoderReader):
    def __init__(self, publish_hz= 10, *args, **kargs ):
        self.tick_data= []
        self.publisher= rospy.Publisher('wheel_rear_ticks', Vector3Stamped, queue_size= 5)
        XenoEncoderReader.__init__(self, *args, **kargs)

    def get_start_byte(self):
        return chr(0xA1)
 
    def get_packet_size(self):
        return 16

    def handle_packet(self):
        left_ticks= self.interpret_tick_byte(5)
        right_ticks= self.interpret_tick_byte(6)
        msg= Vector3Stamped()
        msg.vector.x= left_ticks
        msg.vector.y= right_ticks
        msg.vector.z= 0
        msg.header.frame_id= 'odometry'
        msg.header.stamp= rospy.Time.now()
        self.tick_data.append(msg)

    def publish(self):
        if len(self.tick_data) == 0:            
            return
        for t in self.tick_data:
            self.publisher.publish(t)
        self.tick_data= []
        
        
        
    def spin(self):
        XenoEncoderReader.spin(self)
        self.publish()

def main():
    rospy.init_node('rear_encoders_publisher')
    rep= XenoRearEncoderPublisher(port= '/dev/xeno_rear_encoders')
    rep.connect()
    while not rospy.is_shutdown():
        rep.spin()
    rospy.spin()

        

if __name__ == '__main__':
    main()
