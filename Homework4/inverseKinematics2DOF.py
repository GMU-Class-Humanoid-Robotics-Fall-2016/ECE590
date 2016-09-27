#! /usr/bin/env python


"""





"""

import numpy as np



class ik2DOF(object):

    def __init__( self , x , y , l1 , l2):

        theta2 = np.arccos((x**2 + y**2 - l1**2 - l2**2) / (2*l1 * l2))

        a = y * (l1 + l2 * np.cos(theta2)) - x * (l2*np.sin(theta2))

        b = x * (l1 + l2 * np.cos(theta2)) + y * (l2 * np.sin(theta2))

        theta1 = np.arctan2(a,b)

        self.returnThis( theta1 , theta2 )

    def returnThis(self,theta1,theta2):

        self.theta1 = theta1
        self.theta2 = theta2

        return self

if __name__ == "__main__":

    ik2DOF()