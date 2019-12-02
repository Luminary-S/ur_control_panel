#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")

import rospy
import yaml,os

from PROJECTPATH import *
# from sensor_msgs.msg import Image
# from camFrame import BaseCamFrame
# from sensorFrame import SensorFrame
from frame_node import FrameNode
# from rcrFrame import RCRFrame
# from urFrame import URFrame
from ur_node import URNode
import collections

class guiNode(URNode):
    def __init__(self):
        super(guiNode, self).__init__()
        self.rate = 30
        # self.baseCam = BaseCamFrame()
        # self.sensorAr = SensorFrame()
        # self.rcr = RCRFrame()
        # self.ur = URFrame()
        # self.ur = URNode()
        self.init_pos = []
        self.final_pos = []
        # self.pos_dict = {}
        # d1 = {}
        self.pos_dict = collections.OrderedDict()

    def init(self):
        self.init_node('ur_gui')
        self.loginfo("start ur GUI node...")
        self.node_init()
        # self.define_node_publisher()
        # self.define_node_subscriber()

    # def define_node_subscriber(self):
    #     # self.ur.define_node_subscriber()
    #     pass
    #
    # def define_node_publisher(self):
    #     pass
        # self.ur.define_node_publisher()
        # joint_sub = rospy.Subscriber("/
        # joint_states", JointState, self.callback_joint)
    #
    # """yaml reader"""
    # def params_yaml_reader(self):
    #     fname = YAML_Path + "gui_params.yaml"
    #     f = open(fname)
    #     yamldata = yaml.load(f)
    #     # print yamldata
    #     instrinc_param = yamldata['ins_matrix']
    #     f.close()

    def ur_move_to_point_degree(self, qd, sleepT=4):
        pub = self.joint_pub
        self.ur_movej_to_point(pub, self.getpi(qd))
        rospy.sleep(sleepT)

    def ur_move_to_point_radian(self, qd, sleepT=4):
        pub = self.joint_pub
        self.ur_movej_to_point(pub, qd)
        rospy.sleep(sleepT)

    def ur_step_move_forward(self):
        q = self.ur.get_q()
        # pub = self.joint_pub
        qd = self.ur.ur_step_move_forward(q)
        self.ur_move_to_point_radian( qd, 0)

    def ur_step_move_backward(self):
        q = self.ur.get_q()
        # pub = self.joint_pub
        qd = self.ur.ur_step_move_backward(q)
        self.ur_move_to_point_radian( qd,0)

    def ur_step_move_up(self):
        q = self.ur.get_q()
        # pub = self.joint_pub
        qd = self.ur.ur_step_move_up(q)
        self.ur_move_to_point_radian( qd,0)

    def ur_step_move_down(self):
        q = self.ur.get_q()
        # pub = self.joint_pub
        qd = self.ur.ur_step_move_down(q)
        self.ur_move_to_point_radian( qd,0)

    def ur_step_move_left(self):
        q = self.ur.get_q()
        # pub = self.joint_pub
        qd = self.ur.ur_step_move_left(q)
        self.ur_move_to_point_radian( qd,0)

    def ur_step_move_right(self):
        q = self.ur.get_q()
        # pub = self.joint_pub
        qd = self.ur.ur_step_move_right(q)
        self.ur_move_to_point_radian( qd,0)

    def set_ur_stop(self):
        self.ur.ur_stop()

    def set_ur_type(self, type):
        self.ur.set_UR_ROBOT(type)

    def set_ur_install_type(self, type):
        self.ur.set_UR_install_type(type)


    def loop_move(self, times):
        self.loop_time = times
        i = 0
        pub = self.joint_pub
        # qd = self.ur.ur_step_move_right(q)
        # self.ur_move_to_point(pub, qd)
        qd_list = self.pos_dict.values()
        if not self.pos_dict:  # if no pos in dict
            return False
        while i < self.loop_time:
            i = i+ 1
            for qd in qd_list:
                self.set_moveType('sl')
                self.ur_move_to_point(pub, qd)
                rospy.sleep(5)
        return True

    def add_pos_to_list(self, post_tag):
        q = self.ur.get_q()
        self.pos_dict[post_tag] = q

    def del_pos_from_list(self, pos_tag):
        # q = self.ur.get_q()
        self.pos_dict.pop(pos_tag)

    def get_pos_dict_key(self):
        return self.pos_dict.keys()

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            rate.sleep()

if __name__=="__main__":
    gui = guiNode()
    gui.init_node()
    gui.spin()