#! /usr/bin/env python

"""

input = np.array([theta , alpha  , r , d])




"""

import numpy as np


class dhMatrix(object):


    def __init__(self , input ):

        mat = np.eye(4,dtype=float)
        for i in range(np.shape(input)[0]):

            mat = np.dot(mat,self._createMatrix(input[i,]))

        self._returnThis(mat)


    def _createMatrix(self , cur):


        return np.array([
            [np.cos(cur[0]) , -np.sin(cur[0]) * np.cos(cur[1]) , np.sin(cur[0]) * np.sin(cur[1]) , cur[2] * np.cos(cur[0])],
            [np.sin(cur[0]) , np.cos(cur[0]) * np.cos(cur[1]) , -np.cos(cur[0]) * np.sin(cur[1]) , cur[2] * np.sin(cur[0])],
            [0. , np.sin(cur[1]) , np.cos(cur[1]) , cur[3] ],
            [0.,0.,0.,1.]
        ])


        # return np.array([
        #     [np.cos(cur[0]) , -np.sin(cur[0]) , 0. , cur[2]],
        #     [np.sin(cur[0]) * np.cos(cur[1]) , np.cos(cur[0]) * np.cos(cur[1]) , -np.sin(cur[1]) , -cur[3] * np.sin(cur[1])],
        #     [np.sin(cur[0]) * np.sin(cur[1]) , np.cos(cur[0]) * np.sin(cur[1]) , np.cos(cur[1]) , cur[3] * np.cos(cur[1])],
        #     [0. , 0. , 0. , 1.]
        # ])

    def _returnThis(self,mat):
        self.mat = mat
        return self

if __name__ == "__main__":

    dhMatrix()



