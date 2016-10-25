#! /usr/bin/env python

"""







"""


import numpy as np
from dhMatrix import dhMatrix
from getJacobian import getJacobian
import hubo_ach as ha
import ach

# .12 square

class FKByJacobian(object):

    def __init__(self):


        self.rightCoordinateNames = np.array(['RSP','RSR'])#,'RSY','REB','RWR','RWP'])
        self.leftCoordinateNames = np.array(['LSP','LSR'])#,'LSY','LEB','LWR','LWP'])


        self.rightGoal = np.array([[0.,0.,-360.73],
                                [-138.05 , -127.54 , -307.90] ,
                                   [138.05 , 127.54 , -307.90],
                                   [127.54 , 52.83 , -333.27],
                                   [-37.35 , -270.55 , -235.66]
                                   ])

        self.leftGoal = np.array([[0.,0.,-360.73],
                                    [-138.05 , -127.54 , -307.90] ,
                                   [138.05 , 127.54 , -307.90],
                                   [127.54 , 52.83 , -333.27],
                                   [-37.35 , -270.55 , -235.66]
                                   ])


        self.dhArrayRight = np.array([[0 , -np.pi/2. , 0. , 0.],     # RSP
                                      [0. , 0. , 179.14+181.59 , 0.]])    # RSR


        self.dhArrayLeft = np.array([[0. , -np.pi/2 , 0. , 0. ],  # LSP
                                    [0. , 0. , 179.14+181.59 , 0.]])  # LSR

        deltaTheta = .1
        deltaError = np.array([.1,.1 , .1])

        s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
        r = ach.Channel(ha.HUBO_CHAN_REF_NAME)


        state = ha.HUBO_STATE()
        ref = ha.HUBO_REF()


        [status , framesize] = s.get(state,wait=False,last=True)


        # rightCoordinates = np.zeros([1,np.size(self.rightCoordinateNames)])
        # leftCoordinates = np.zeros([1,np.size(self.leftCoordinateNames)])
        errorOk = .01


        ##### MAIN ALGORITHM HERE #####

        running = True


        goal = 0

        while running:

            self._getCurrentTheta(state , s) # Get Current Location

            rightError = self._getMet('R',self.rightGoal,goal)
            leftError = self._getMet('L',self.leftGoal,goal)


            while rightError > errorOk and leftError > errorOk:

                rightJacobian = getJacobian(self.dhArrayRight,deltaTheta).jacobian
                leftJacobian = getJacobian(self.dhArrayLeft,deltaTheta).jacobian

                rightJacobianInverse = np.linalg.pinv(rightJacobian)
                leftJacobianInverse = np.linalg.pinv(leftJacobian)

                rightCurrent = self._getFK(self.dhArrayRight)
                leftCurrent = self._getFK(self.dhArrayLeft)

                rightDeltaError = self._getNext(rightCurrent,self.rightGoal,deltaError,'R',goal)
                leftDeltaError = self._getNext(leftCurrent,self.leftGoal,deltaError,'L',goal)


                # print np.shape(rightDeltaError) , np.shape(rightJacobianInverse)
                # print rightJacobianInverse

                rightDeltaTheta = np.squeeze(np.dot(rightDeltaError,rightJacobianInverse))
                leftDeltaTheta = np.squeeze(np.dot(leftDeltaError,leftJacobianInverse))

                # for i in range(np.shape(self.dhArrayRight)[0]):
                #     self.dhArrayRight[i,0] += rightDeltaTheta[i]
                #
                # for i in range(np.shape(self.dhArrayLeft)[0]):
                #     self.dhArrayLeft[i,0] += leftDeltaTheta[i]


                print rightDeltaTheta

                print self.dhArrayRight

                self._sendMoveCommand(s,r,ref,state , rightDeltaTheta + self.dhArrayRight[:,0] ,
                                      leftDeltaTheta + self.dhArrayLeft[:,0])

            goal += 1

            if goal > np.shape(self.rightGoal)[0]:
                goal = 1

    def _sendMoveCommand(self , s, r, ref, state ,right , left):
        currentValues = self._returnDOFValues(state , s)

        print right
        print left

        ref.ref[ha.RSR] = right[1]
        ref.ref[ha.RSP] = right[0]

        ref.ref[ha.LSR] = left[1]
        ref.ref[ha.LSP] = left[0]

        r.put(ref)

        self._simulationDelay( 1. , state , s )

    def _getNext(self,current , goal , de , side , goalIteration):


        h = self._getMet(side , goal , goalIteration )
        current = np.squeeze(current)
        dx = (goal[goalIteration,0] - current[0]) * (de[0]/h)
        dy = (goal[goalIteration,1] - current[1]) * (de[1]/h)
        dz = (goal[goalIteration,2] - current[2]) * (de[2]/h)

        # print dx , dy , dz
        # print dx
        return np.reshape(np.array([[dx],[dy],[dz]]),[1,3])


    def _getMet(self,side,goal,goalIteration):

        if side == 'R':

            current = self._getFK(self.dhArrayRight)

        elif side == 'L':

            current = self._getFK(self.dhArrayLeft)

        current = np.squeeze(current)

        return np.sqrt((current[0] - goal[goalIteration,0]) ** 2 + (current[1] - goal[goalIteration,1]) ** 2 + (current[2] - goal[goalIteration,2])**2)

    def _getFK(self,curArray):

        mat = dhMatrix(curArray).mat
        mat = np.dot(mat[0:3,0:3],mat[0:3,3])

        return np.array([mat])

    def _getCurrentTheta(self , state , s):

        out = self._returnDOFValues(state , s)

        for i in range(np.size(self.rightCoordinateNames)):

            if self.rightCoordinateNames[i] == 'RSP':
                self.dhArrayRight[i,0] = out[0]
            elif self.rightCoordinateNames[i] == 'RSR':
                self.dhArrayRight[i,0] = out[1]
            elif self.rightCoordinateNames[i] == 'RSY':
                self.dhArrayRight[i,0] = out[2]
            elif self.rightCoordinateNames[i] == 'REB':
                self.dhArrayRight[i,0] = out[3]
            elif self.rightCoordinateNames[i] == 'RWY':
                self.dhArrayRight[i,0] = out[4]
            elif self.rightCoordinateNames[i] == 'RWP':
                self.dhArrayRight[i,0] = out[5]
            elif self.rightCoordinateNames[i] == 'RHY':
                self.dhArrayRight[i,0] = out[13]
            elif self.rightCoordinateNames[i] == 'RHR':
                self.dhArrayRight[i,0] = out[14]
            elif self.rightCoordinateNames[i] == 'RHP':
                self.dhArrayRight[i,0] = out[15]
            elif self.rightCoordinateNames[i] == 'RKN':
                self.dhArrayRight[i,0] = out[16]
            elif self.rightCoordinateNames[i] == 'RAP':
                self.dhArrayRight[i,0] = out[17]
            elif self.rightCoordinateNames[i] == 'RAR':
                self.dhArrayRight[i,0] = out[18]
            else:
                print "UGH ERROR IN RIGHT DH-ARRAY THETA ALLOCATION"


        for i in range(np.size(self.leftCoordinateNames)):

            if self.leftCoordinateNames[i] == 'LSP':
                self.dhArrayLeft[i,0] = out[6]
            elif self.leftCoordinateNames[i] == 'LSR':
                self.dhArrayLeft[i,0] = out[7]
            elif self.leftCoordinateNames[i] == 'LSY':
                self.dhArrayLeft[i,0] = out[8]
            elif self.leftCoordinateNames[i] == 'LEB':
                self.dhArrayLeft[i,0] = out[9]
            elif self.leftCoordinateNames[i] == 'LWY':
                self.dhArrayLeft[i,0] = out[10]
            elif self.leftCoordinateNames[i] == 'LWP':
                self.dhArrayLeft[i,0] = out[11]
            elif self.leftCoordinateNames[i] == 'LHY':
                self.dhArrayLeft[i,0] = out[19]
            elif self.leftCoordinateNames[i] == 'LHR':
                self.dhArrayLeft[i,0] = out[20]
            elif self.leftCoordinateNames[i] == 'LHP':
                self.dhArrayLeft[i,0] = out[21]
            elif self.leftCoordinateNames[i] == 'LKN':
                self.dhArrayLeft[i,0] = out[22]
            elif self.leftCoordinateNames[i] == 'LAP':
                self.dhArrayLeft[i,0] = out[23]
            elif self.leftCoordinateNames[i] == 'LAR':
                self.dhArrayLeft[i,0] = out[24]
            else:
                print "UGH ERROR IN LEFT DH-ARRAY THETA ALLOCATION"





    def _sendStartingMoveCommand(self , s, r, ref, state, pitch , increment):
        currentValues = self._returnDOFValues(state , s)

        moving=True
        inc = 0
        while moving:

            self._announceHandLocation(state, s)


            ref.ref[ha.RHP] = rHipInc[inc]
            ref.ref[ha.LHP] = lHipInc[inc]


            r.put(ref)

            self.simulationDelay(.2,state,s)
            inc += 1
            if inc >= increment:
                moving = False






    def _getIK(l1,l2,theta,current,goal,desiredError,dtheta,de):

        # current = _getFK(theta,l1,l2)
        returnTheta = []
        while(_getMet(current , goal)) > desiredError:

            J = _getJ(theta,dtheta,l1,l2)

            Ji = np.linalg.pinv(J)

            deltaE = _getNext(current , goal , de)
            deltaTheta = Ji * deltaE
            theta = theta + deltaTheta
            returnTheta.append(theta)
            current = _getFK(theta , l1 , l2)

        return returnTheta


    def _simulationDelay( self, waitTime , state , s ):
        [statuss, framesizes] = s.get(state, wait=False, last=False)
        current = state.time
        while (True):
            [statuss, framesizes] = s.get(state, wait=True, last=False)

            if ((state.time - current) > waitTime):
                break

    def _announceHandLocation(self,state,s):

        currentValues = self._returnDOFValues(state , s)

        if leg=='RIGHT':
            height = self.l1*np.cos(currentValues[17]) + self.l2*np.cos(currentValues[17] + currentValues[16]) + self.l3*np.cos(currentValues[17] + currentValues[16] + currentValues[15])
        elif leg=='LEFT':
            height = self.l1 * np.cos(currentValues[23]) + self.l2 * np.cos(currentValues[22] + currentValues[23]) + self.l3 * np.cos(currentValues[21] + currentValues[23] + currentValues[22])

        print "\t\t\tCURRENT HIP HEIGHT IS {}".format(height)

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


if __name__ == "__main__":
    FKByJacobian()