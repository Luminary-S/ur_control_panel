#!/usr/bin/venv python
# -*- coding: utf-8 -*-

# Copyright (c) 2018.10, Guangli SUN
# All rights reserved.

import rospy,time

from ur3_kinematics import Kinematic
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from jacobian import ur_robot
import numpy as np
import numpy.linalg as lg
import math
import matplotlib.pyplot as pl
import sys
from trans_methods import *
import tf

class UR():
    def __init__(self, num):
#        self.port = ''
        self.qstart = [ 92.24,-135.31,-99.37, 226.82, -87.84, 3.86 ]#[ 91.48, -202.73, 70.01, 136.88, -88.94, 180.24 ]
        self.ur_num = num
        self.q = self.qstart
        self.ur_status = 'urstop'
        self.weights = [1.] * 6
        self.radius = 0.22
        # weights = [1.] * 6
        self.cont = 15
        self.now_ur_pos = getpi(self.qstart)
        self.now_vel = [0,0,0,0,0,0]
        self.init_q = self.qstart
        self.final_q = self.init_q
        self.ur_init_ready = 0
        self.ur_final_ready = 0
        self.ur_ready = 0
        self.urtype = "ur3"
        self.install_type = "normal"
        self.kin = Kinematic()
        self.jac = ur_robot(self.urtype,6)
        self.STEP = 0.002 # unit: m        
        self.moveType = "stop"
        # self.step_move = {
        #     "up": self.ur_step_move_up,
        #     "down": self.ur_step_move_down,
        #     "back": self.ur_step_move_backward,
        #     "forward": self.ur_step_move_forward,
        #     "left": self.ur_step_move_left,
        #     "right": self.ur_step_move_right
        # }


    """kinematics related """
    def set_UR_ROBOT(self,type):
        self.kin.set_urtype(type)
        self.urtype = type

    def set_UR_install_type(self, type):
        self.install_type = type

    def get_urobject_urkine(self):
        ur0 = Kinematic()
        return ur0

    def get_IK_from_T(self,T,q_last):
        ur0 = self.kin
        return ur0.best_sol( self.weights, q_last, T )

    def solve_q(self, q, T):
        # rad
        qd = self.kin.best_sol(self.weights, q, T)
        return qd

    def getJac(self,q):
        return self.jac.dk(q)

    """q related """
    def get_qstart(self):
        return self.qstart
    
    def get_q(self):
        print("get_q,",self.now_ur_pos)
        return self.now_ur_pos
    
    def get_T(self):
        return self.kin.Forward(self.now_ur_pos)
    
    def get_pos(self):
        T = self.get_T()
        return [T[3],T[7], T[11]]

    def set_T(self,q):
        return self.kin.Forward(q)
    
    def get_init_q(self):
        q = getDegree( self.init_q )
        return q
    def set_init_q(self):
        self.init_q = self.now_ur_pos
        return self.init_q
        
    def get_final_q(self):
        q = getDegree(self.final_q)
        return q
    def set_final_q(self):
        self.final_q = self.now_ur_pos
        return self.final_q

    def get_ee_pos_cartesian(self):
        T = self.get_T()
        T = np.array(T)
        T = T.reshape((4,4))
        R = tr2r(T)
        L = (transl(T)*1000).T.tolist()[0]
        theta = r2Euler(R)
        cartesian_pos = L+theta
        cartesian_pos = [round(x,3) for x in cartesian_pos ]
        return cartesian_pos

    # def get_

    def set_step(self, step):
        self.STEP = step

    def ur_step_q(self, q, F_T):
        # step designed joints rotation
        urk = self.kin
        F_T = urk.Forward(q)
        qd = self.solve_q(q, F_T)
        return qd

    def ur_step_move(self, q , i, direction):
        # step cartersian space, end effector movement
        # print(" enter calculation....")
        # print("q:",q)
        urk = self.kin
        F_T = urk.Forward(q)
        print("before F_t:", F_T)
        print("before up and down: ",F_T[11])
        # TransT = self.get_T_translation(F_T)
        print("========step=====", self.STEP)
        F_T[i] = F_T[i] + direction*self.STEP   
        qd = self.solve_q(q, F_T)
        T = self.kin.Forward(qd)
        print("up and down: ",T[11])
        # T = self.
        # print("qd",qd)
        # sys.exit(0)
        return qd

    def ur_step_joint_move(self, q, i, angle):
        delta = self.getpi(angle)
        qd = q
        qd[i] = q[i] + delta
        return qd

    # def ur_step_vel_move(self, vel):
    #     urdk = self.jac
    #     vx = 0
    #     vy = 0
    #     vz = -v_list[i] #vz = v_imp
    #     theta_x = 0
    #     theta_y = 0
    #     theta_z = 0
    #     v = [vx,vy,vz,theta_x,theta_y,theta_z]
    #     q = self.get_q()
    #     print("q:", q)
    #     J = ur.dk(q)
    #     print("J:", J)
    #     v = np.mat(v)
    #     inv_J = lg.inv(J)
    #     print("inv J:", inv_J)
    #     qdot_d = np.dot(inv_J, v.T).T
    #     qdot_d = qdot_d.tolist()[0]
    #     return v, qdot_d

    def ur_xyz_move(self,q,xyz_list):
        #up:x; left:y; back:z
        [x,y,z] = xyz_list
        urk = self.kin
        F_T = urk.Forward(q)
        F_T[11] = F_T[11] - x        
        F_T[3] = F_T[3] + y
        F_T[7] = F_T[7] - z
        # F_T[i] = F_T[i] + direction*self.STEP  
        qd = self.solve_q(q, F_T)
        return qd

    def ur_xyz_move_backup(self,q,xyz_list):
        #up:x; left:y; back:z
        [x,y,z] = xyz_list
        urk = self.kin
        F_T = urk.Forward(q)
        F_T[11] = F_T[11] - x        
        F_T[3] = F_T[3] + y
        F_T[7] = F_T[7] - z
        # F_T[i] = F_T[i] + direction*self.STEP  
        Rt = self.TtoR(F_T)
        Pt = self.TtoL(F_T)
        # Rt= np.array(Rt)
        # Pt = np.array(Pt)
        # q = np.array(q)
        qd = self.jac.ik(Rt,Pt,q)
        # qd = qd.tolist()
        # qd = self.solve_q(q, F_T)
        return qd

    def TtoR(self, T):
        R = [ [ T[0], T[1], T[2] ], [ T[4], T[5], T[6] ], [ T[8], T[9], T[10] ] ]
        return R
    def TtoL(self, T):
        L = [ T[3], T[7], T[11] ]
        return L

    def ur_step_move_up(self, q):
        print("set direction...")
        return self.ur_step_move(q, 11, -1)
    
    def ur_step_move_down(self, q):
        return self.ur_step_move(q, 11, 1)
    
    def ur_step_move_left(self, q):
        return self.ur_step_move(q, 3, -1)
    
    def ur_step_move_right(self, q):
        return self.ur_step_move(q, 3, 1)
    
    def ur_step_move_forward(self, q):
        return self.ur_step_move(q, 7, -1)
    
    def ur_step_move_backward(self, q):
        return self.ur_step_move(q, 7, 1)
    

    def get_T_translation(self, T):
        trans_x = T[3]
        trans_y = T[7]
        trans_z = T[11]
        return [trans_x, trans_y, trans_z]

  

    def set_vel(self, t, T_list,vmax, amax):
        v0 = 0
        v1 = 0
        # vmax = 0.16
        # amax = 1
        [t1, t2, t3] = T_list
        # t = i * del_t
        # accelarate up
        if t >= 0 and t < t1:
            v = v0 + t * amax
        # accelarate fixed
        elif t >= t1 and t < t2:
            # if t == t1:
            #     v1 = vmax
            v = vmax
        # accelarate down
        elif t >= t2 and t < t3:
            # if t == t2:
            #     v2 = vmax
            v = vmax - amax * (t - t2)
        else:
            v = 0
        print("v:",v)
        return v


    def vel_planning(self,del_t):
        # del_t = 0.001
        Jmax = 10
        amax = 0.5
        t_list = self.T_planning(amax,Jmax)
        T_sum = 0
        T_list = []
        for i in iter(t_list) :
            T_sum = T_sum + int(i/del_t)
            T_list.append(int(i/del_t))
        [t1, t2, t3, t4, t5, t6, t7] = T_list
        print("T_list",T_list)
        # t1 = 5*del_t
        # t2 = 10*del_t
        # t3 = 15*del_t
        # t4 = 15*del_t
        # t5 = 20 * del_t
        # t6 = 25 * del_t
        # t7 = 30 * del_t
        vnow = 0
        v_list = []
        T_sum = 65
        for i in range(0,T_sum,1):
            t = i * del_t
            # accelarate up
            if  t >=0 and t <t1:
                v = 0.5 * Jmax * t * t
            # accelarate fixed
            elif t >=t1 and t <t2:
                if t == t1:
                    v1 = v
                v = v1 + amax * (t-t1)
            # accelarate down
            elif t >=t2 and t <t3:
                if t == t2:
                    v2 = v
                v = v2 + amax * (t-t2) - 0.5 * Jmax * (t - t2) * (t - t2)
            # fixed velocity
            elif t >=t3 and t <t4:
                if t == t3:
                    v3 = v
                v = v3
            elif t >= t4 and t < t5:
                if t == t4:
                    v4 = v
                v = v4 - 0.5 * Jmax * (t - t4) * (t - t4)
            elif t >= t5 and t < t6:
                if t == t5:
                    v5 = v
                v = v5  + amax * (t-t5)
            elif t >= t6 and t < t7:
                if t == t6:
                    v6 = v
                v = v6 + amax * (t - t6) - 0.5 * Jmax * (t - t6) * (t - t6)
            v_list.append(v)
        return v_list


    def demo_vel_move(self):
        '''
        ee coodinate:
        v[0] positive: right (people view, forward to wall)
        v[1] positive: window back
        v[2] negative: up  positive: down
        v[3] positive: pitch up (yang); negative: pitch down (fu)
        v[4] positive: roll right (right gundong); negative: roll left (left gundong)
        v[5] positive: yaw clockwise (right turn); negative: yaw counter-clockwise (left turn)
        '''
        ur = ur_robot("ur3", 6)
        

        num = 2
        v0 = 0
        v1 = 0
        l0 = 0
        l1 = 0.12
        vmax = 0.16
        amax = 1
        v_list = []
        t_list = self.t_planning_simple(0, 0.08, v0,v1, vmax,amax)
        # v_list = self.vel_planning_simple(0,0.1,0,0, 0.15)
        print(t_list)
        # t =
        # pl.plot(t, v_list,'-')
        # pl.show()
        # sys.exit(0)
        q_init = [85.85,-117.86,-89.94,200.49,-94.08,3.38]
        # self.ur_movej_to_point(self.pub, getpi(q_init))
        rospy.sleep(5)
        # sys.exit(0)

        t0 = time.time()
        t2 = t0
        # t1 = t0
        # tt = t1 - t2
        qdot_d = [0,0,0,0,0,0]
        while True:
            # v = [0, 0, -0.1, 0, 0,0]
            # t1 = time.time()
            self.urscript_speedj_pub(self.pub, qdot_d, 0.2, 0.5)
            t2_old = t2
            t2 = time.time()
            tt = t2 - t2_old
            delta_t = t2 - t0 + tt
            print("det_t:",delta_t)
            # if delta_t > 0.5:
            #     sys.exit(0)
            v_y = self.set_vel(delta_t,t_list, vmax, amax)
            # sys.exit(0)
            v_list.append(-v_y)
            v = [0,0, -v_y, 0,0,0]
            q = self.get_q()
            # print("q:",q)
            J = ur.dk(q)
            print("J:", J)

            v = np.mat(v)
            inv_J = np.mat(J).I
            # sys.exit(0)
            # print("inv J:", inv_J)
            qdot_d = np.dot( inv_J, v.T ).T
            qdot_d = qdot_d.tolist()[0]
            # print(type(qdot_d))
            print("qdot_d:", qdot_d)
            if v_y == 0:
                break
        t = range(0,len(v_list),1)
        pl.plot(t, v_list,'-')
        pl.show()
    

    
    def vel_move(self, direction, v_list):
        ur = ur_robot("ur3", 6)
        for i in range(0, len(v_list), 0):
            vx = 0
            vy = 0
            vz = -v_list[i] #vz = v_imp
            theta_x = 0
            theta_y = 0
            theta_z = 0
            v = [vx,vy,vz,theta_x,theta_y,theta_z]
            q = self.get_q()
            print("q:", q)
            J = ur.dk(q)
            print("J:", J)
            v = np.mat(v)
            inv_J = lg.inv(J)
            print("inv J:", inv_J)
            qdot_d = np.dot(inv_J, v.T).T
            qdot_d = qdot_d.tolist()[0]
            print(type(qdot_d))
            print("qdot_d:", qdot_d)
            # self.urscripte_speedj_pub(self.pub, qdot_d, 0.5, 0.5)
            # rospy.sleep(0.1)
    def step_vel_move(self, vel, direction):
        pass

    def path_planning_cartesian(self):
        # point = []
        # ur = ur_robot("ur3", 6)
        # #1, get q, T, J
        # q = self.get_q()
        # T = self.get_T()
        # J = ur.dk(q)
        #2, interpolation of up and down lines
        v0 = 0
        v1 = 0
        l0 = 0
        l1 = 0.8
        vmax = 0.2
        amax = 1
        p_list1 = self.interpolation_line(l0,l1, v0,v1, vmax, amax)

        #3, interpolation of left and right lines
        v0 = 0
        v1 = 0
        l0 = 0
        l1 = 0.15
        vmax = 0.2
        amax = 1
        p_list2 = self.interpolation_line(l0, l1, v0, v1, vmax, amax)
        return p_list1,p_list2

    def interpolation_line(self, l0,l1, v0,v1, vmax, amax):
        # cnt =500
        p_list = []
        del_l = l1 - l0
        t_list = self.t_planning_simple(l0,l1,v0,v1,vmax,amax)
        delta_t = 0.01
        cnt = int(t_list[-1] / delta_t)
        # the differential of the line trajectory is an isosceles trapezoid(deng yao ti xing)
        [t1, t2, t3] = [int(t/delta_t) for t in iter(t_list)]
        print("t1,t2,t3",t1,t2,t3)
        if t1 != t2:
            for t in range(0,cnt+1):
                # t = i * del_t
                # accelarate up
                if t >= 0 and t < t1:
                    # t = t/delta_t
                    ld =  0.5 * amax * t * t * delta_t * delta_t
                # accelarate fixed
                elif t >= t1 and t < t2:
                    if t == t1:
                        ld1 = ld
                    # t = t / delta_t
                    ld = ld1 +  vmax * (t - t1) * delta_t
                # accelarate down
                elif t >= t2 and t <= t3:
                    # t = t / delta_t
                    if t == t2:
                        ld2 = ld
                    ld = ld2 + 0.5*(vmax + ( vmax - amax * (t-t2)* delta_t))*(t-t2)*delta_t
                else:
                    ld = 0
                p_list.append(ld)
        else:
            for t in range(0, cnt + 1):
                # t = i * del_t
                # accelarate up
                if t >= 0 and t < t1:
                    # t = t/delta_t
                    ld = 0.5 * amax * t * t * delta_t * delta_t
                # accelarate fixed
                # elif t >= t1 and t < t2:
                #     if t == t1:
                #         ld1 = ld
                #     # t = t / delta_t
                #     ld = ld1 + vmax * (t - t1) * delta_t
                # accelarate down
                elif t >= t2 and t <= t3:
                    # t = t / delta_t
                    if t == t2:
                        ld2 = ld
                    ld = ld2 + 0.5 * (amax * t2 + ( amax *t2 - amax * (t - t2) ))* delta_t * (t - t2) * delta_t
                else:
                    ld = 0
                p_list.append(ld)
        print("p_list:", p_list)
        print("plast:",p_list[-1])
        t = range(0, len(p_list), 1)
        pl.plot(t, p_list, '-')
        pl.show()
        # sys.exit(0)
        return p_list


    def urscript_pub(self, pub, qq, vel, ace, t):
        ss = "movej([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(qq[3]) + "," + str(qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
        # ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
        pub.publish(ss)

    def urscript_speedj_pub(self, pub , vel, ace, t):
        ss = "speedj([" + str(vel[0]) + "," + str(vel[1]) + "," + str(vel[2]) + "," + str(vel[3]) + "," + str(
            vel[4]) + "," + str(vel[5]) + "]," + "a=" + str(ace) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
        # ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
        pub.publish(ss)

def getpi(l):
    res = []
    print(l)
    for i in l:
        res.append( round( i / 180 * math.pi , 3) )
    return res

def getDegree(l):
    res = []
    for i in l:
        res.append( round( i*180 / math.pi , 3) )
    return res

def test_ur():
    qstart = [76.02, -160.79, -77.66, 227.14, -102.12, 264.76]
    q0 = getpi(qstart)
    q = [-0.24116307893861944, -0.9259565512286585, 1.8004541397094727, -0.8985760847674769, 1.0013269186019897, 4.849152565002441]
    # ur = UR(0)
    print(getDegree(q))
    # qd = ur.ur_step_move(q0, 11,-1)


if __name__=="__main__":
    test_ur()
