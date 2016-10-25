#! /usr/bin/env python

"""




input = np.array([theta , alpha  , r , d])

"""



import numpy as np
from dhMatrix import dhMatrix

class getJacobian(object):

    def __init__(self , dhArray , deltaTheta):

        self.jacobian = np.zeros([np.shape(dhArray)[0],3],dtype=float)

        for i in range(np.shape(dhArray)[0]):

            cur = dhArray
            cur[i,0] += deltaTheta

            out = dhMatrix(cur).mat
            out = np.dot(out[0:3,0:3] , out[0:3,3])



            self.jacobian[i,] = out


    def _returnThis(self):

        return self



if __name__ == "__main__":

    getJacobian()





    def _return(self):
        print 'asdf'