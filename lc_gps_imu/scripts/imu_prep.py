#!/usr/bin/env python

'''

@author Jose Alvarez-Ruiz
@date   June 2016

'''

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import sys
import tf
import tf.transformations as tlib
import math

class IMUPrep:
    def __init__(self, imu_topic, odom_topic, onlyCov= False):
        rospy.Subscriber(imu_topic, Imu, self.imuCallback)
        rospy.Subscriber(odom_topic, Odometry, self.odometryCallback)
        self.imu_publisher= rospy.Publisher('imu_prep', Imu, queue_size= 10)
        self.odometry_publisher= rospy.Publisher('odom_prep', Odometry, queue_size= 10)
        self.imu_ready= False
        self.orientation= tlib.quaternion_from_euler(0, 0, 0)
#        self.prev_imu_time= None
        # IMU linear accelerations
        self.all_laccs= []
        self.lacc_offsets= None
        # IMU angular velocities
        self.all_avels= []
        self.avel_offsets= None
        self.onlyCov= onlyCov
        
        

###     def collectImu(self, lacc):
###         if len(self.all_laccs) < 1000:
###             self.all_laccs.append(lacc)
###             return
###         if self.lacc_offsets:
###                 return
###         l= np.array(self.all_laccs)
### 
###         # http://stackoverflow.com/questions/11686720/is-there-a-numpy-builtin-to-reject-outliers-from-a-list
###         d= np.abs(l - np.median(l, axis= 0))
###         dmed= np.median(d)
###         s = d / dmed  #if dmed else 0.0
###         self.lacc_offsets= \
###           [np.mean(l[s[:, i] < 2, i]) for i in range(3)]
### 

    
    def collect(self, curr, acum, offsets):
        if len(acum) < 200:
            acum.append(curr)
            return None
        if offsets:
         return offsets
        l= np.array(acum)
        # Distances w.r.t. median (per column)
        d= np.abs(l - np.median(l, axis= 0))
        dmed= np.median(d)
        s= (d + 0.0000000001) / (dmed + 0.00000001) # radio of distance w.r.t median distance
        cov= np.cov(l, rowvar= False)
        if self.onlyCov:
            print cov
        offsets= \
          [np.mean(l[s[:, i] < 2, i]) for i in range(3)]
        return offsets

    def imuCallback(self, msg):
#        if not self.prev_imu_time:
 #           self.prev_imu_time= msg.header.stamp
 #           return
        tmp= msg.linear_acceleration
        lacc= (tmp.x, tmp.y, tmp.z)
        tmp= msg.angular_velocity
        avel= (tmp.x, tmp.y, tmp.z)
        if self.onlyCov and  self.lacc_offsets:
            print 'Linear accelearation covariance matrix:'
        self.lacc_offsets= self.collect(lacc, self.all_laccs, self.lacc_offsets)
        if self.onlyCov and self.avel_offsets:
            print 'Angular velocity covariance matrix:'
            self.onlyCov= False
        self.avel_offsets= self.collect(avel, self.all_avels, self.avel_offsets)
        if not self.lacc_offsets:
            return 
        corrected= [lacc[i] - self.lacc_offsets[i]
                    for i in range(3)]
        corrected_msg= msg
        corrected_msg.linear_acceleration.x= corrected[0]
        corrected_msg.linear_acceleration.y=     corrected[1]
        corrected_msg.linear_acceleration.z=     corrected[2]
        sa= 20
        corrected_msg.angular_velocity_covariance= [sa, 0.0, 0.0,
                                                    0.0, sa, 0.0,
                                                    0.0, 0, sa]
        sl= 0.0005
        corrected_msg.linear_acceleration_covariance= [sl, 0.0, 0.0,
                                                       0.0, sl, 0.0,
                                                       0.0, 0., sl]



        
#        dt= (msg.header.stamp - self.prev_imu_time).to_sec()
#        dr= tlib.quaternion_from_euler(corrected_msg.angular_velocity.x * dt,
#                                        corrected_msg.angular_velocity.y * dt,
#                                        corrected_msg.angular_velocity.z * dt)

 #       new_orientation= tlib.quaternion_multiply(dr, self.orientation)
        
  #      m= math.sqrt(new_orientation[0]**2 +
  #                  new_orientation[1]**2 +
  #                  new_orientation[2]**2 +
  #                  new_orientation[3]**2)
  ##      new_orientation= new_orientation / m
  #      corrected_msg.orientation.x= new_orientation[0] 
  #      corrected_msg.orientation.y= new_orientation[1] 
  #      corrected_msg.orientation.z= new_orientation[2] 
  #      corrected_msg.orientation.w= new_orientation[3]
  #      self.orientation= new_orientation
        self.imu_publisher.publish(corrected_msg)
        self.imu_ready= True
   #     self.prev_imu_time= msg.header.stamp

    def odometryCallback(self, msg):
        if self.imu_ready:
            self.odometry_publisher.publish(msg)


if __name__ == '__main__':
    ip= IMUPrep('imu', 'odom',  True)
    rospy.init_node('imu_preprocessing')
    rospy.spin()
