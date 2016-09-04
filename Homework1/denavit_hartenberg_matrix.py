#! /usr/bin/env python

"""

For information on the Denavit-Hartenberg Matrix see the following:
    https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    http://www.fit.hcmup.edu.vn/~hungnv/teaching/Robotics/0471649902_-_Robot_Modeling_and_Control.pdf

Inputs:
    d: offset along previous 'z' to the common normal
    theta: angle about previous 'z' from old 'x' to new 'x'
    a: length of the common normal.  Assuming a revolute joint, this is the radius about previous 'z'
    alpha: angle about common normal, from old 'z' axis to new 'z' axis


"""

import numpy
import argparse

class dhMatrix(object):

    def __init__(self , d , theta , a , alpha):

        self.T = numpy.array([
            [numpy.cos(theta) , -numpy.sin(theta) * numpy.cos(alpha) , numpy.sin(theta) * numpy.sin(alpha) , a * numpy.cos(theta)],
            [numpy.sin(theta) , numpy.cos(theta) * numpy.cos(alpha) , -numpy.cos(theta) * numpy.sin(alpha) , a * numpy.sin(theta)],
            [0. , numpy.sin(alpha) , numpy.cos(alpha) , d],
            [0. , 0. , 0. , 1.]
                    ])

        self._correctForArctan()

        self._returnThis()

    def _correctForArctan(self):

        self.arcTanPhi = numpy.arctan(self.T[1,0] / self.T[0,0])

        signX = numpy.sign(self.T[0,3])
        signY = numpy.sign(self.T[1,3])

        if signX == 1 and signY == 1:
            self.circularPhi = self.arcTanPhi
        if signX == 1 and signY == -1:
            self.circularPhi = (2. * numpy.pi) + self.arcTanPhi
        if signX == -1 and signY == -1:
            self.circularPhi = numpy.pi + self.arcTanPhi
        if signX == -1 and signY == 1:
            self.circularPhi = numpy.pi + self.arcTanPhi


    def _returnThis(self):

        return self



class dhMatrixZ(object):

    def __init__(self , d , theta):

        self.Z = numpy.array([
            [numpy.cos(theta) , -numpy.sin(theta) , 0. , 0.],
            [numpy.sin(theta) , numpy.cos(theta) , 0. , 0.],
            [0. , 0. , 1. , d],
            [0. , 0. , 0. , 0.]
        ])

        self._returnThis()

    def _returnThis(self):

        return self

class dhMatrixX(object):

    def __init__(self , a , alpha):

        self.X = numpy.array([
            [1., 0., 0., a],
            [0., numpy.cos(alpha) , -numpy.sin(alpha) , 0.],
            [0., numpy.sin(alpha) , numpy.cos(alpha) , 0.],
            [0.,0.,0.,1.]
        ])

        self._returnThis()

    def _returnThis(self):

        return self

if __name__ == "__main__":


    parser = argparse.ArgumentParser(description='Denavit-Hartenberg Matrix Equation', epilog='\n\n CORRECT USAGE = \n\n python denavit_hartenberg_matrix.py -d 0. -t .78 , -r 1 , -a .78 ')
    parser.add_argument('-d','--Offset',type=float,help="offset along previous 'z' to the common normal")
    parser.add_argument('-t','--Theta',type=float,help="angle about previous 'z' from old 'x' to new 'x'")
    parser.add_argument('-r','--Length',type=float,help="length of the common normal.  Assuming a revolute joint, this is the radius about previous 'z'")
    parser.add_argument('-a','--Alpha',type=float,help="angle about common normal, from old 'z' axis to new 'z' axis")

    args = parser.parse_args()

    out = dhMatrix(args.Offset , args.Theta , args.Length , args.Alpha)
    print out.T , out.arcTanPhi , out.circularPhi