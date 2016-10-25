#! /usr/bin/env python

"""





"""


import numpy as np
import hubo_ach as ha
import ach


class jacobianForward(object):

    def __init__(self):

        ### Standard Intro ###

        s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
        r = ach.Channel(ha.HUBO_CHAN_REF_NAME)


        state = ha.HUBO_STATE()
        ref = ha.HUBO_REF()


        [status , framesize] = s.get(state,wait=False,last=True)

        self.rightCoordinateNames = np.array(['RSP','RSR' ,'RSY','REB','RWR','RWP'])
        self.leftCoordinateNames = np.array(['LSP','LSR' ,'LSY','LEB','LWR','LWP'])
        sizes = np.array([])

        deltaTheta = .1
        error = 5
        step = 10

        leftGoal = np.array([])
        rightGoal = np.array([])


    def _getJacobian(self,side,deltaTheta,state,s):
        out = np.zeros([3,np.size(self.rightCoordinateNames)])

        for i in range(np.shape(out)[0]):
            for j in range(np.shape(out)[1]):



    def _getFK(self , thetaUpdate , update, side , state , s):


        if side == 'L':

            currentTheta = self._getCurrentTheta(state,s,'L')

        elif side == 'R':

            currentTheta = self._getCurrentTheta(state , s ,'R')

        else:
            print "error"

        if update == True:

            currentTheta += thetaUpdate


        T0 = np.eye(4)
        T0[1,3] = self.sizes([0])

        T1 = np.eye(4)


        T2 = np.eye(4)


        T3 = np.eye(4)
        T3[2,3] = self.sizes([1])

        T4 = np.eye(4)
        T4[2,3] = self.sizes([2])

        T5 = np.eye(4)

        out = np.dot(self._pitch(currentTheta[0]) , T0)
        out = np.dot(out,np.dot(self._roll(currentTheta[1]),T1))
        out = np.dot(out,np.dot(self._yaw(currentTheta[2]),T2))
        out = np.dot(out,np.dot(self._pitch(currentTheta[3],T3)))
        out = np.dot(out,np.dot(self._roll(currentTheta[4]),T4))
        return np.dot(out,np.dot(self._pitch(currentTheta[5]),T5))



    def _roll(self,theta):

        return np.array([
            [1. , 0. , 0. , 0.],
            [0. , np.cos(theta) , -np.sin(theta) , 0.],
            [0. , np.sin(theta) , np.cos(theta) , 0.],
            [0.,0.,0.,1.]
        ])



    def _pitch(self,theta):

        return np.array([
            [np.cos(theta) , 0. , np.sin(theta) , 0.],
            [0. , 1. , 0. , 0.],
            [-np.sin(theta) , 0. , np.cos(theta) , 0.],
            [0.,0.,0.,1.]
        ])

    def _yaw(self,theta):

        return np.array([
            [np.cos(theta) , -np.sin(theta) , 0.],
            [np.sin(theta) , np.cos(theta) , 0.],
            [0. , 0. , 1. , 0.],
            [0.,0.,0.,1.]
        ])

    def _simulationDelay( self, waitTime , state , s ):
        [statuss, framesizes] = s.get(state, wait=False, last=False)
        current = state.time
        while (True):
            [statuss, framesizes] = s.get(state, wait=True, last=False)

            if ((state.time - current) > waitTime):
                break

    def _getCurrentTheta(self , state , s , side ):

        out = self._returnDOFValues(state , s)

        currentTheta = np.zeros([np.size(self.rightCoordinateNames),1])
        if side == 'R':
            for i in range(np.size(self.rightCoordinateNames)):

                if self.rightCoordinateNames[i] == 'RSP':
                    currentTheta[i] = out[0]
                elif self.rightCoordinateNames[i] == 'RSR':
                    currentTheta[i] = out[1]
                elif self.rightCoordinateNames[i] == 'RSY':
                    currentTheta[i] = out[2]
                elif self.rightCoordinateNames[i] == 'REB':
                   currentTheta[i]  = out[3]
                elif self.rightCoordinateNames[i] == 'RWY':
                    currentTheta[i] = out[4]
                elif self.rightCoordinateNames[i] == 'RWP':
                   currentTheta[i]  = out[5]
                elif self.rightCoordinateNames[i] == 'RHY':
                   currentTheta[i]  = out[13]
                elif self.rightCoordinateNames[i] == 'RHR':
                   currentTheta[i]  = out[14]
                elif self.rightCoordinateNames[i] == 'RHP':
                   currentTheta[i]  = out[15]
                elif self.rightCoordinateNames[i] == 'RKN':
                   currentTheta[i]  = out[16]
                elif self.rightCoordinateNames[i] == 'RAP':
                   currentTheta[i]  = out[17]
                elif self.rightCoordinateNames[i] == 'RAR':
                   currentTheta[i]  = out[18]
                else:
                    print "UGH ERROR IN RIGHT DH-ARRAY THETA ALLOCATION"

        if side == 'L':
            for i in range(np.size(self.leftCoordinateNames)):

                if self.leftCoordinateNames[i] == 'LSP':
                   currentTheta[i]  = out[6]
                elif self.leftCoordinateNames[i] == 'LSR':
                   currentTheta[i]  = out[7]
                elif self.leftCoordinateNames[i] == 'LSY':
                   currentTheta[i]  = out[8]
                elif self.leftCoordinateNames[i] == 'LEB':
                   currentTheta[i]  = out[9]
                elif self.leftCoordinateNames[i] == 'LWY':
                   currentTheta[i]  = out[10]
                elif self.leftCoordinateNames[i] == 'LWP':
                   currentTheta[i]  = out[11]
                elif self.leftCoordinateNames[i] == 'LHY':
                   currentTheta[i]  = out[19]
                elif self.leftCoordinateNames[i] == 'LHR':
                   currentTheta[i]  = out[20]
                elif self.leftCoordinateNames[i] == 'LHP':
                  currentTheta[i]   = out[21]
                elif self.leftCoordinateNames[i] == 'LKN':
                  currentTheta[i]   = out[22]
                elif self.leftCoordinateNames[i] == 'LAP':
                  currentTheta[i]   = out[23]
                elif self.leftCoordinateNames[i] == 'LAR':
                  currentTheta[i]   = out[24]
                else:
                    print "UGH ERROR IN LEFT DH-ARRAY THETA ALLOCATION"

        return currentTheta

    def _returnDOFValues(self , state , s):
        [statuss, framesizes] = s.get(state, wait=False, last=False)
        return np.array([
        [state.joint[ha.RSP].pos],
        [state.joint[ha.RSR].pos],
        [state.joint[ha.RSY].pos],
        [state.joint[ha.REB].pos],
        [state.joint[ha.RWY].pos],
        [state.joint[ha.RWP].pos],
        [state.joint[ha.LSP].pos],
        [state.joint[ha.LSR].pos],
        [state.joint[ha.LSY].pos],
        [state.joint[ha.LEB].pos],
        [state.joint[ha.LWY].pos],
        [state.joint[ha.LWP].pos],
        [state.joint[ha.WST].pos],
        [state.joint[ha.RHY].pos],
        [state.joint[ha.RHR].pos],
        [state.joint[ha.RHP].pos],
        [state.joint[ha.RKN].pos],
        [state.joint[ha.RAP].pos],
        [state.joint[ha.RAR].pos],
        [state.joint[ha.LHY].pos],
        [state.joint[ha.LHR].pos],
        [state.joint[ha.LHP].pos],
        [state.joint[ha.LKN].pos],
        [state.joint[ha.LAP].pos],
        [state.joint[ha.LAR].pos]
        ])




