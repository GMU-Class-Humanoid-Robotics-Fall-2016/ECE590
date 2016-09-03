#! /usr/bin/env python

"""

This program calculates the end effector position in a three degree of freedom situation.

Inputs:
    -L1 <or> --Length1: the length (any unit) of the first arm
    -L2 <or> --Length2: the length (any unit) of the second arm
    -L3 <or> --Length3: the length (any unit) of the third arm
    -A1 <or> --Angle1: the angle (radian) of the first joint
    -A2 <or> --Angle2: the angle (radian) of the second joint
    -A3 <or> --Angle3: the angle (radian) of the third joint

Outputs:

    x: the x position of the end effector
    y: the y position of the end effector
    phi: the angle (radian) of the end effector

Usage:

    <user>$ python degree_of_freedom.py -L1 1 -L2 1 -L3 1 -A1 .2 -A2 .2 -A3 .2
            x = 2.72646318675 , y = 1.1527301465 , phi = 0.6

TODO: create plot function for this


"""


import numpy
import argparse
import matplotlib.pyplot as plt


class threeDOF(object):

    def __init__(self,args):

        x = args.Length1 * numpy.cos(args.Angle1) + args.Length2 * numpy.cos(args.Angle1 + args.Angle2) + args.Length3 * numpy.cos(args.Angle1 + args.Angle2 + args.Angle3)
        y = args.Length1 * numpy.sin(args.Angle1) + args.Length2 * numpy.sin(args.Angle1 + args.Angle2) + args.Length3 * numpy.sin(args.Angle1 + args.Angle2 + args.Angle3)
        phi = args.Angle1 + args.Angle2 + args.Angle3

        print "x = {} , y = {} , phi = {}".format(x,y,phi)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='3 DOF Simulation With End Effector Location', epilog='\n\n CORRECT USAGE = \n\n python eh.py -L1 5 -L2 6 -L3 7')
    parser.add_argument('-L1','--Length1',type=float,help="The length of the first arm")
    parser.add_argument('-L2','--Length2',type=float,help="The length of the second arm")
    parser.add_argument('-L3','--Length3',type=float,help="The length of the third arm")
    parser.add_argument('-A1','--Angle1',type=float,help="The angle of the first arm in radians")
    parser.add_argument('-A2','--Angle2',type=float,help="The angle of the second arm in radians")
    parser.add_argument('-A3','--Angle3',type=float,help="The angle of the third arm in radians")
    #
    #
    # print 'done', Length1
    args = parser.parse_args()

    threeDOF(args)
