#!/usr/bin/python3
import collections
import copy
import os
import time
import argparse

import numpy as np
import rospy
import rosnode

import tensorflow as tf
from agiros_msgs.msg import Command
from agiros_msgs.msg import QuadState
from std_msgs.msg import Empty, Float32
from scipy.spatial.transform import Rotation as R
import pyquaternion
import yaml
import pandas as pd


class AgilePilot:
    def __init__(self):
        print("Initializing agile_pilot_node...")
        rospy.init_node('agile_pilot_node', anonymous=False)

        self.ctrl_mode = "CTBR"
        self.publish_commands = False
        self.max_coll_thrust = 20.0
        self.max_bodyrate_xy = 6.0
        self.max_bodyrate_z = 3.0

        quad_name = 'kingfisher'

        self.start_sub = rospy.Subscriber("/" + quad_name + "/start_navigation", Empty, self.start_callback,
                                          queue_size=1, tcp_nodelay=True)
        self.odom_sub = rospy.Subscriber("/" + quad_name + "/agiros_pilot/state", QuadState, self.state_callback,
                                         queue_size=1, tcp_nodelay=True)
        self.img_sub = rospy.Subscriber("/" + quad_name + "/agiros_pilot/state", Image, self.img_callback,
                                        queue_size=1, tcp_nodelay=True)

        if self.ctrl_mode == "CTBR" or self.ctrl_mode == "SRT":
            self.cmd_pub = rospy.Publisher("/" + quad_name + "/agiros_pilot/feedthrough_command", Command, queue_size=1)
        else:
            # TODO: implement option for velocity commands
            pass

    def img_callback(self, data):
        # TODO
        print("Received image!")
        pass

    def state_callback(self, data):
        start_time_callback = time.time()

        position = np.array([data.pose.position.x,
                             data.pose.position.y,
                             data.pose.position.z], dtype=np.float32)
        attitude = np.array([data.pose.orientation.w,
                             data.pose.orientation.x,
                             data.pose.orientation.y,
                             data.pose.orientation.z], dtype=np.float32)
        velocity = np.array([data.velocity.linear.x,
                             data.velocity.linear.y,
                             data.velocity.linear.z], dtype=np.float32)
        omega = np.array([data.velocity.angular.x,
                          data.velocity.angular.y,
                          data.velocity.angular.z], dtype=np.float32)

        # TODO: Candidates need to provide code that computes actions here

        action = np.array([0.5, 0.0, 0.0, 0.0])

        if self.ctrl_mode == "CTBR":
            coll_thrust = self.max_coll_thrust * action[0]
            bodyrate_x = (2.0 * action[1] - 1.0) * self.max_bodyrate_xy
            bodyrate_y = (2.0 * action[2] - 1.0) * self.max_bodyrate_xy
            bodyrate_z = (2.0 * action[3] - 1.0) * self.max_bodyrate_z

            cmd_msg = Command()
            cmd_msg.t = data.t
            cmd_msg.header.stamp = data.header.stamp
            cmd_msg.is_single_rotor_thrust = False
            cmd_msg.collective_thrust = coll_thrust
            cmd_msg.bodyrates.x = bodyrate_x
            cmd_msg.bodyrates.y = bodyrate_y
            cmd_msg.bodyrates.z = bodyrate_z

            if self.publish_commands:
                self.cmd_pub.publish(cmd_msg)
        else:
            assert False, "unsupported control mode"

        print("Command computation took %.1f ms" % (1000.0 * (time.time() - start_time_callback)))

    def start_callback(self, data):
        print("Start publishing commands!")
        self.publish_commands = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Agile Pilot.')

    args = parser.parse_args()
    agile_pilot = AgilePilot()
    rospy.spin()
