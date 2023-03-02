#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Shafeek Zakko}
# {student id}
# {shafeek@kth.se}

from dubins import *
import math
import random


class Vertex():

    def __init__(self,x,y,theta = 0,phi = 0, closest = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.closest = closest
        self.step = 0
        return


def find_closest(vertices, xin,yin):
    closest = None
    d1 = 9999
    for node in vertices:
        d = dist_calculator(node.x, node.y, xin, yin)
        if d < d1:
            closest = node
            d1 = d
            continue
    return closest
        

#Checking collosion with obstacles and bounds

def check_coll(car, xin, yin):
    for i in car.obs:
        d = dist_calculator(i[0],i[1],xin,yin)
        if d <= i[2]:
            return True
    if not (car.xlb < xin < car.xub) or not (car.ylb < yin < car.yub):
        return True
    return False


#checking if point is the destination
def check_goal(car, xin, yin):
    l = dist_calculator(xin, yin, car.xt, car.yt)
    if l  < 0.5:
        return True
    else:
        return False


def dist_calculator(xt,yt,xin,yin):
    
    d = (xt-xin)**2 + (yt-yin)**2

    return d

def solution(car):

    ''' <<< write your code below >>> '''
    vertices = []

    # initial state 
    init_node = Vertex(car.x0, car.y0,0, 0)

    vertices.append(init_node)


    while True:
        xr = random.uniform(car.xlb + 0.2, car.xub - 0.2)
        yr = random.uniform(car.ylb + 0.2, car.yub - 0.2)

        closest = find_closest(vertices, xr,yr)
        if closest is None:
            continue

        phi = math.atan2(yr - closest.y, xr - closest.x) - closest.theta
        if phi < -1*math.pi/4:
            phi = -1*math.pi/4
        if phi > math.pi/4:
            phi = math.pi/4

        xn = closest.x
        yn = closest.y
        thetan = closest.theta
        for i in range(50):
            xTmp = xn
            yTmp = yn
            thetaTmp = thetan
            xn, yn, thetan = step(car,xn, yn, thetan, phi)
            if check_coll(car, xn , yn):
                xn = xTmp                
                yn = yTmp
                thetan = thetaTmp
                break
            if check_goal(car, xn, yn):
                break
        
        
        if check_goal(car, xn, yn):
            final_node = Vertex(car.xt, car.yt,phi= phi, closest = closest)
            final_node.step = i
            break

        if xn == closest.x or yn == closest.y or i == 0:
            continue
                
        new_node = Vertex(xn, yn, thetan,phi, closest)
        new_node.step = i
        vertices.append(new_node)


    path = [final_node]
    reversed_controls = [final_node.phi]
    reversed_steps = [final_node.step]
    controls=[]
    times=[0]

    while path[-1].closest is not None:

        reversed_controls.append(path[-1].closest.phi)
        reversed_steps.append(path[-1].closest.step)
        path.append(path[-1].closest)

    controls = list(reversed(reversed_controls[:-1]))
    steps = list(reversed(reversed_steps[:-1]))
    for i in range(len(steps)):
        times.append(times[-1] + steps[i] * 0.01)

    ''' <<< write your code below >>> '''
    
    return controls, times
