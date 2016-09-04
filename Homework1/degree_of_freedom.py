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
    -M  <or> --Method: Method to run, 1: forward kinematics, 2: Denavit-Hartenberg Matrix

Outputs:

    x: the x position of the end effector
    y: the y position of the end effector
    phi: the angle (radian) of the end effector

Usage:

    <user>$ python degree_of_freedom.py -L1 1 -L2 1 -L3 1 -A1 .2 -A2 .2 -A3 .2 -M 1
            x = 2.72646318675 , y = 1.1527301465 , phi = 0.6

TODO: create plot function for this


"""


import numpy
import argparse
import matplotlib.pyplot as plt
from denavit_hartenberg_matrix import dhMatrix
# from rotation_matrix import rotationMatrix

class threeDOF(object):

    def __init__(self , Length1 , Length2 , Length3 , Angle1 , Angle2 , Angle3 , Method=1):

        if Method==1:

            x = Length1 * numpy.cos(Angle1) + Length2 * numpy.cos(Angle1 + Angle2) + Length3 * numpy.cos(Angle1 + Angle2 + Angle3)
            y = Length1 * numpy.sin(Angle1) + Length2 * numpy.sin(Angle1 + Angle2) + Length3 * numpy.sin(Angle1 + Angle2 + Angle3)
            phi = Angle1 + Angle2 + Angle3

            self.threeDOF = numpy.array([x , y , phi])

        elif Method == 2:

            out1 = dhMatrix(0.,Angle1,Length1,0.)
            out2 = dhMatrix(0.,Angle2,Length2,0.)
            out3 = dhMatrix(0.,Angle3,Length3,0.)

            out2_1 = numpy.dot(out1.T,out2.T)
            self.out = numpy.dot(out2_1,out3.T)

            self.arcTanPhi = numpy.arctan(self.out[1,0] / self.out[0,0])

            signX = numpy.sign(self.out[0,3] - out2_1[0,3])
            signY = numpy.sign(self.out[1,3] - out2_1[1,3])

            if signX == 1 and signY == 1:
                self.circularPhi = self.arcTanPhi
            if signX == 1 and signY == -1:
                self.circularPhi = (2. * numpy.pi) + self.arcTanPhi
            if signX == -1 and signY == -1:
                self.circularPhi = numpy.pi + self.arcTanPhi
            if signX == -1 and signY == 1:
                self.circularPhi = numpy.pi + self.arcTanPhi

        else:
            print "error, incorrect Method"

        self._returnThis()

    def _returnThis(self):

        return self




if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='3 DOF Simulation With End Effector Location', epilog='\n\n CORRECT USAGE = \n\n python degree_of_freedom.py -L1 5 -L2 6 -L3 7')
    parser.add_argument('-L1','--Length1',type=float,help="The length of the first arm")
    parser.add_argument('-L2','--Length2',type=float,help="The length of the second arm")
    parser.add_argument('-L3','--Length3',type=float,help="The length of the third arm")
    parser.add_argument('-A1','--Angle1',type=float,help="The angle of the first arm in radians")
    parser.add_argument('-A2','--Angle2',type=float,help="The angle of the second arm in radians")
    parser.add_argument('-A3','--Angle3',type=float,help="The angle of the third arm in radians")
    parser.add_argument('-M','--Method',type=int,help="Methodology To Use, 1: Direct Kinematics , 2: Denavit-Hartenberg Matrix",default=1)
    #
    #
    # print 'done', Length1
    args = parser.parse_args()

    out = threeDOF(args.Length1 , args.Length2 , args.Length3 , args.Angle1 , args.Angle2 , args.Angle3 , args.Method)

    print 'Method = ', args.Method

    if args.Method == 2:
        print "matrix output = \n",out.out
        print "phi (from x axis is) ",out.circularPhi
        print "phi (from last point) is ",out.arcTanPhi
    else:
        print "x = {} , y = {} , phi = {}".format(out.threeDOF[0] , out.threeDOF[1] , out.threeDOF[2])
