#!/usr/bin/env python

'''

@author Jose Alvarez-Ruiz
@date   August 2016

'''

import rospy
import actionlib
from fub_sound.msg import SayAction, SayGoal, SayResult, SayFeedback

class SayClient:
    def __init__(self, volume= 100, language= 'en', dummy_mode= False):
        self.volume= volume
        self.language= language
        self.dummy_mode= dummy_mode

        if not self.dummy_mode:
            print 'Waiting for say action server...'
            self.client= actionlib.SimpleActionClient('/say', SayAction)
            self.client.wait_for_server()
            print 'Done...'

    def feedback_cb(self, fb):
        pass

    def done_cb(self, state, result):
        self.done= True

    def block(self):
        rate= rospy.Rate(10)
        while True:
            rate.sleep()
            if self.done == True:
                break

    def __call__(self, text, volume= None, language= None, block= True):
        if self.dummy_mode:
            print 'SAY: ', text
            return
        if volume is None: volume= self.volume
        if language is None: language= self.language
        goal= SayGoal()
        goal.text= text
        goal.language= language
        goal.volume= volume
        self.done= False
        self.client.send_goal(goal, feedback_cb= self.feedback_cb,
                              done_cb= self.done_cb)
        
        if block:
            self.block()
