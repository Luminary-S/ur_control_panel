#!usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
# from hand_in_eye_X import *


def tr2jac( T, samebody = 1):
    #T = np.array(T)
    R = tr2r(T)
    #jac = np.zeros((6, 6))
    """
    jac = [ jac_part1,  jac_part2;
            jac_part3,  jac_part4;
                ]
    """
    if samebody:
        jac_part1 = R.T
        jac_part2 = np.dot( skew( transl(T) ), R).T
        jac_part3 = np.zeros((3,3))
        jac_part4 = R.T

    else:
        jac_part1 = R.T
        jac_part2 = np.zeros((3,3))
        jac_part3 = np.zeros((3,3))
        jac_part4 = R.T
    jac_row1 = np.column_stack( (jac_part1, jac_part2) )
    jac_row2 = np.column_stack( (jac_part3, jac_part4) )
    jac = np.row_stack( (jac_row1, jac_row2) )
    return jac

# '''  new tr to jac from Yuelinzhu'''
# def tr2jac( T, samebody=1):
#     # T = np.array(T)
#     R = tr2r(T)
#     # jac = np.zeros((6, 6))
#     """
#     jac = [ jac_part1,  jac_part2;
#             jac_part3,  jac_part4;
#                 ]
#     """
#     if samebody == 1:
#         jac_part1 = R
#         New_trans = np.dot(-1 * (R.I), transl(T))
#         jac_part2 = -np.dot(R, skew(New_trans))
#         # print "self.transl(T))",self.transl(T)
#         # T1=[1,2,3]
#         # print "self.skew(self.transl(T))\n",self.skew(New_trans)
#         jac_part3 = np.zeros((3, 3))
#         jac_part4 = R
#
#     else:
#         jac_part1 = R
#         jac_part2 = np.zeros((3, 3))
#         jac_part3 = np.zeros((3, 3))
#         jac_part4 = R
#     jac_row1 = np.column_stack((jac_part1, jac_part2))
#     jac_row2 = np.column_stack((jac_part3, jac_part4))
#     jac = np.row_stack((jac_row1, jac_row2))
#     return jac

"""
if l is 3*1 , then get 
skew(l) = [ 0, -l(2), l(1)
            l(2), 0 , -l(0)
            -l(1), l(0), 0]
if l is 1*1, then get
skew(l) = [ 0 , -l[0]
            l[0], 0 ]

"""
def skew(l):
    a, b = np.shape(l)
    try:
        if a == 3:
            s = np.array( [ 0, -l[2], l[1], l[2], 0, -l[0], -l[1], l[0], 0 ] )
            s = s.reshape((3,3))
            # print "s:", s
            return s
        elif a == 1:
            s = np.array( [ 0, -l[0], l[0], 0])
            s = s.reshape( (2,2) )
            return s
    except:
        print("erro l size!!!  3*1 or 1*1 required!")


def tr2r(T):
    r = [ 0, 1, 2]
    c = [ 0, 1, 2]
    R1 = T[r]
    R = R1[:,c]
    return R

def r2Euler(R):
    # R = np.zeros((3, 3), dtype=np.float64)
    # cv2.Rodrigues(rvecs, R)
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    #print('dst:', R)

    x = x*180.0/3.141592653589793
    y = y*180.0/3.141592653589793
    z = z*180.0/3.141592653589793
    return [x,y,z]

def transl(T):
    r = [3]
    c = [0 , 1, 2]
    l1 = T[:, r]
    l = l1[c]
    return l

def getpi(l):
    res = []
    print(l)
    for i in l:
        res.append( round( i / 180 * math.pi , 3) )
    return res

def test_main():
    pass
#     # 1, get the X matrix
#     X = get_ur3_X()
    T = [0.9999984136498173, -0.001593286546933248, -0.0007963264582434141, -0.030323900530061416, -0.0007975939724302236, -0.000795057176226172, -0.9999993658637698, 0.15849393784669805, 0.0015926524115072896, 0.9999984146597762, -0.0007963267107332633, 0.36721240413295503, 0.0, 0.0, 0.0, 1.0]
    # T =[0.9960126681698078, -0.01280973257091012, 0.08828746001965403, 0.38741541064998, 0.0858056866643338, -0.1332965279718059, -0.987354758821126, 0.013471574131938863, 0.024416162297183575, 0.9909934138944231, -0.1316658825837364, 0.15685088139545816, 0.0, 0.0, 0.0, 1.0]
    T = [0.9960143719908139, -0.01267746789299043, 0.08828732975669139, 0.3872240590681055, 0.08583506594660784, -0.1328028369622896, -0.987418729794361, -0.012057159963503046, 0.02424277710341921, 0.9910613948200131, -0.13118536296296243, 0.11688352779317447, 0.0, 0.0, 0.0, 1.0]
    T = np.array(T)
    X = T.reshape((4, 4))
    R = tr2r(X)
    import tf
    import tf2_ros
    q = tf.transformations.quaternion_from_matrix(X)
    L = transl(X).T.tolist()[0]
    theta = r2Euler(R)
    theta2 = tf.transformations.euler_from_quaternion(q)
    print("rotation:", R)
    print("transition:", L)
    # jac = tr2jac(X)
    # print("jac:", jac)
    print("theta",theta )
    print("thetha2",theta2)
    print("tehta in radian:", getpi(theta))
    print("tehta2 in radian:", getpi(theta2))
    # print("cartesian", L+theta)

    # a, b =np.shape(X[:,3])
    # print X[:,3]
    # print a,b

    # 2, get  the  samebody transfer matrix to jacobian matrix
    #jac = tr2jac(X)
    #print "jac", jac


if __name__=="__main__":
    test_main()