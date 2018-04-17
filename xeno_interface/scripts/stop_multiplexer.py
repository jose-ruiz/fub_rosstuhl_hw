#!/usr/bin/env python

'''
@author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
@date   August 2016
'''

import rospy
from std_msgs.msg import Bool 


source_topics= ['stop/emergency_button',
                'stop/system_stop',
                'stop/obstacle_stop',
                'stop/user_stop',
                'stop/remote_stop']


class StopMultiplex:
    def __init__(self, source_topics, wait= 1):
        self.publisher= rospy.Publisher('global_stop', Bool, queue_size= 5)
        self.last_true= None
        self.last_time= None
        self.wait= wait
        for t in source_topics:
            print 'Subscribing to ', t
            rospy.Subscriber(t, Bool, self.callback)
        
    def callback(self, msg):
        now= rospy.Time.now()
        self.last_time= now        
        if msg.data:
            self.last_true= now

    def stopped(self):
        now= rospy.Time.now()
        if  self.last_time is None:
            print '#No stop agent active'
            return True
        dt= now - self.last_time        
        if dt.to_sec() > 0.2:
            print '#No stop agent active'
            # No stop signal available... better stop
            return True
        if self.last_true is None:
               return False
        dt= now - self.last_true
        return dt.to_sec() < self.wait

    def publish(self):
        msg= Bool()
        msg.data= self.stopped()
        self.publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('stop_multiplexer')
    sm= StopMultiplex(source_topics)
    rate= rospy.Rate(20)
    while not rospy.is_shutdown():
        sm.publish()
        rate.sleep()
    rospy.spin()
