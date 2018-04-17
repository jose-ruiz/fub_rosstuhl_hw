#!/usr/bin/env python

'''

@author Jose Alvarez-Ruiz
@date   August 2016

'''

import rospy
import actionlib
from fub_sound.msg import SayAction, SayGoal, SayResult, SayFeedback
import time
import subprocess

class SayServer:
    def __init__(self):
        pass
        self.server= actionlib.SimpleActionServer('/say',
                                                  SayAction,
                                                  self.action_cb,
                                                  False)
        self.pitch= 30
        self.speed= 140
        self.delay= 3
        self.server.start()            

    def create_command(self, goal):
        format= 'espeak -a %d -v%s -p %d -s %d -g %d "%s"'        
        return format  %  (goal.volume,
                            goal.language,
                            self.pitch,
                            self.speed,
                            self.delay,
                            goal.text)
    
    def action_cb(self, goal):
        print 'action arrived'
        result= SayResult()
        cmd= self.create_command(goal)
        print cmd
        p= subprocess.Popen(cmd, shell= True)
        p.wait()
        self.server.set_succeeded(result, 'Said done!')
        
if __name__ == '__main__':
    rospy.init_node('rosstuhl_sound_server')
    say_server= SayServer()
    rospy.spin()
