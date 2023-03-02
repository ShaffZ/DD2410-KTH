#! /usr/bin/env python3

import math

"""
    # {Shafeek Zakko}
    # {Shafeek@kth.se}
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """

    l0, l1, l2 = 0.07, 0.30, 0.35

    x = x - l0 # removing offset

    r2 = (x**2 + y**2) #squared distance from origo
    cos_q1 = (r2 - l1**2 - l2**2) / (2 * l1 * l2) #cosinus 
    
    # calculating the second link angle
    q1 = math.acos(cos_q1) #inverse cosinus 


    # calculating the first link angle
    q0 = math.atan2(y , x) - math.atan2(l2 * math.sin(q1) , l1 + l2 * cos_q1)


    q[0] = q0
    q[1] = q1
    q[2] = z
    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q
