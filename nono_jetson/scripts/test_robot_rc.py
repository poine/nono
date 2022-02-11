#!/usr/bin/env python
import platform, signal, sys
import math, numpy as np
import rospy, sensor_msgs
from sensor_msgs.msg import Joy

import time, serial, struct
import nono_jetson.md25 as md25
import common_vision.utils

class Node:
    def __init__(self, rate=50):
        self.md25 = md25.MD25()
        rospy.init_node('nono_{}'.format('jetson_rc'))
        rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.joy_callback)
        self.lin_vel, self.rot_vel = 0, 0
        self.lmot, self.rmot = 0, 0
        self.rate = rospy.Rate(rate)
        self.sat = 127
        
    def run(self):
         last = rospy.get_rostime() - self.rate.sleep_dur
         while not rospy.is_shutdown():
            now = rospy.get_rostime()
            dt = now - last; last = now
            self.md25.read()
            #print(self.md25.enc_l, self.md25.enc_r)
            self.md25.write(self.lmot, self.rmot)

            
            self.rate.sleep()
         
    def joy_callback(self, msg):
        self.last_input = rospy.get_rostime()
        kl, ka = 30., -15.
        self.lin_vel, self.rot_vel = msg.axes[1], msg.axes[2]
        self.lmot = int(np.clip(kl*self.lin_vel + ka*self.rot_vel, -self.sat, self.sat))
        self.rmot = int(np.clip(kl*self.lin_vel - ka*self.rot_vel, -self.sat, self.sat))
        #print('{:.1f} {:.1f}'.format(self.lin_vel, self.rot_vel))
        
def main():
    n = Node()
    n.run()

if __name__ == '__main__':
    np.set_printoptions(precision=7, suppress=True, linewidth=200)
    main()
