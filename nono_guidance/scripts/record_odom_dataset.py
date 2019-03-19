#!/usr/bin/env python
import time, math, numpy as np, sys
import rospy, rospkg, sensor_msgs.msg, nav_msgs.msg

import julie_misc.utils as jmu

class Node:
    def __init__(self):
        rospy.init_node('record_odometry')

        # joint state
        rospy.Subscriber('/nono/joint_states', sensor_msgs.msg.JointState, self.join_state_callback)
        self.lw_angle, self.rw_angle = [], []
        self.odom_stamp = []

        # truth
        rospy.Subscriber('/nono/base_link_truth', nav_msgs.msg.Odometry, self.gazebo_truth_callback)
        self.truth_pos = []
        self.truth_ori = []
        self.truth_lvel = []
        self.truth_rvel = []
        self.truth_stamp = []

    def gazebo_truth_callback(self, msg):
        self.truth_pos.append( jmu.list_of_xyz(msg.pose.pose.position))
        self.truth_ori.append( jmu.list_of_xyzw(msg.pose.pose.orientation))
        self.truth_lvel.append( jmu.list_of_xyz(msg.twist.twist.linear))
        self.truth_rvel.append( jmu.list_of_xyz(msg.twist.twist.angular))
        self.truth_stamp.append(msg.header.stamp.to_sec())
 
    def join_state_callback(self, msg):
        self.lw_angle.append(msg.position[0])
        self.rw_angle.append(msg.position[1])
        self.odom_stamp.append(msg.header.stamp.to_sec())

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print('recorded {} odometry and {} thruth'.format( len(self.odom_stamp), len(self.truth_stamp)))
            rate.sleep()

    def save(self, filename):
        print('saving to {}'.format(filename))
        np.savez(filename,
                 encoders_lw = np.array(node.lw_angle),
                 encoders_rw = np.array(node.rw_angle),
                 encoders_stamp = np.array(node.odom_stamp),
                 truth_pos   = np.array(node.truth_pos),
                 truth_ori   = np.array(node.truth_ori),
                 truth_lvel  = np.array(node.truth_lvel),
                 truth_rvel  = np.array(node.truth_rvel),
                 truth_stamp = np.array(node.truth_stamp))
        
if __name__ == '__main__':
    node = Node()
    try:
        node.run()
    except rospy.ROSInterruptException:
        print('recorded {} odometry and {} thruth'.format( len(node.odom_stamp), len(node.truth_stamp)))
    output_filename = '/tmp/nono_io_1' if len(sys.argv) < 2 else sys.argv[1]
    node.save(output_filename)
