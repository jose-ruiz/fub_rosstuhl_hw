#!/usr/bin/env python

'''
@author Jose Alvarez-Ruiz
@date   May 2016
'''
#rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link imu 100
import rospy
import serial
import math
import sys
from sensor_msgs.msg import Imu
from nmea_msgs.msg import Sentence

import tf
import tf.transformations as tlib

class GPSIMUPublisher:
    def __init__(self, port, baudrate, *args, **kargs):
        self.port= port
        self.baudrate= baudrate
        self.last_imu= None
        self.imu_publisher= rospy.Publisher('imu_raw', Imu, queue_size= 5)
        self.gps_publisher= rospy.Publisher('nmea_sentence', Sentence, queue_size= 5)
        self.imu_seq= 0
        self.gps_seq= 0

    def connect(self):
        self.serial= serial.Serial(self.port, self.baudrate)

    def publish_nmea(self, s):
        s= s[:-2] # Remove carriage return and line feed chars
        msg= Sentence()
        msg.header.frame_id= 'gps'
        msg.header.stamp= rospy.Time.now()
        msg.header.seq+= 1
        msg.sentence= s
        self.gps_seq+=1
        self.gps_publisher.publish(msg)

    def publish_imu(self, s):        
        tokens= s[1:-1].split(',')
        values= map(float, tokens[1:])
        dtms= int(tokens[0])
        if not self.last_imu:
            self.last_imu= values
            return
        angv= map(lambda vp: ((vp[0] - vp[1]) / (dtms / 1000.0)) * math.pi / 180.0,
                zip(values[0:3], self.last_imu[0:3] ))
        linacc= map(lambda v: v / 8192.0 * 9.81, values[3:])
        # Create IMU message
        msg= Imu()
        msg.header.stamp= rospy.Time.now()
        msg.header.frame_id= 'imu'
        msg.header.seq= self.imu_seq
        self.imu_seq+= 1
        q= tlib.quaternion_from_euler(0, 0, 0)
        msg.orientation.x= q[0]
        msg.orientation.y= q[1]
        msg.orientation.z= q[2]
        msg.orientation.w= q[3]
        msg.angular_velocity.x= -angv[2]
        msg.angular_velocity.y= angv[1]
        msg.angular_velocity.z= -angv[0]
        msg.linear_acceleration.x= -linacc[0]
        msg.linear_acceleration.y= -linacc[1]
        msg.linear_acceleration.z= linacc[2]
        msg.orientation_covariance= [-1.0, 0.0, 0.0,
                                     0.0, -1.0, 0.0,
                                     0.0, 0, -1.0]
        self.imu_publisher.publish(msg)
        self.last_imu= values
        
    def publish(self):
        l= self.serial.readline()[:-1]
        if l.startswith('#'):
            print l
        elif l.startswith('$'):
            self.publish_nmea(l)
        elif l.startswith('!'):            
            self.publish_imu(l)

def main():
    pass


if __name__ == '__main__':
    print 'GPS NMEA string and IMU data publisher...'
    p= GPSIMUPublisher(port= '/dev/xeno_gps_imu', baudrate= 115200)
    p.connect()
    rospy.init_node('lc_gps_imu')    
    while not rospy.is_shutdown():
        p.publish()
