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

        self.rightCoordinateNames = np.array(['RSP','RSR' ,'RSY','REB','RWY','RWP'])
        self.leftCoordinateNames = np.array(['LSP','LSR' ,'LSY','LEB','LWY','LWP'])


        self.sizes = np.array([[300.38+300.03+303.87-181.87],[179.14],[181.59]])

        deltaTheta = .1
        error = 5
        step = 30

        print "\n\n\nWELCOME TO INVERSE KINEMATICS"

        print "CENTERING ARMS AND CALCULATION BOX BASED ON THIS POSITION.\n"
        print "THIS SIMULATION RUNS SLOW BECAUSE IT IS DOING THE PLANNING ONLINE"
        print "INSTEAD OF DOING IT A PRIORI.\n"
        print "THIS MEANS IT IS TREATING THE NATURAL PHYSICS AS A DISTURBANCE"
        print "WHICH MAKES THE CONTROL SLOWER."
        print "IF THE CONTROL IS NOT SET CORRECTLY THEN SINGULARITIES ARE FOUND"
        print "AND THINGS LIKE ARMS GO BACKWARDS AROUND IT TO TRY AND SOLVE THE STEP."
        print "\n\nENJOY..."


        self._raiseArmsToCenter(state,s,r,ref)


        self._simulationDelay(1,state,s)

        left = self._getFK(0,False,'L',state,s)
        right = self._getFK(0,False,'R',state,s)


        leftGoal = np.array([[left[0]+50., left[1] , left[2] + 60.],
                             [left[0]+50., left[1] , left[2] -60.],
                             [left[0]-50., left[1] , left[2] - 60.],
                             [left[0]-50., left[1] , left[2] + 60.]])


        rightGoal = np.array([[right[0] + 50. , right[1] , right[2] + 50.],
                              [right[0] + 50. , right[1] , right[2] - 50.],
                              [right[0] - 50. , right[1] , right[2] - 50.],
                              [right[0] - 50. , right[1] , right[2] + 50.]])



        run = True

        runningCount = 0

        goal = 0

        print "RUNNING ALGORITHM FOR 4 GOALS"

        while run:

            leftError = self._error(leftGoal[goal,],'L',state,s)
            rightError = self._error(rightGoal[goal,],'R',state,s)
            print "\n\nGOAL NUMBER {} \n\n".format(goal)
            print "LEFT HAND GOAL POSITION IS: {}".format(leftGoal[goal,])
            print "RIGHT HAND GOAL POSITION IS: {}".format(rightGoal[goal,])
            print "\n\n"
            print '\tLeft Error = {} , Right Error = {}'.format(round(leftError, 3),round(rightError, 3))


            while leftError > error or rightError > error:
                if runningCount == 50:
                    print '\tLeft Error = {} , Right Error = {}'.format(round(leftError,3),round(rightError,3))
                    runningCount = 0

                lSideJacobian = self._getJacobian('L',deltaTheta,state,s)
                rSideJacobian = self._getJacobian('R',deltaTheta,state,s)

                lSideJacobian = np.linalg.pinv(lSideJacobian)
                rSideJacobian = np.linalg.pinv(rSideJacobian)

                leftNextStep = self._nextStep(step,leftError,state,s,'L',leftGoal[goal,])
                rightNextStep = self._nextStep(step,rightError,state,s,'R',rightGoal[goal,])

                lThetaUpdate = np.dot(lSideJacobian,leftNextStep)
                rThetaUpdate = np.dot(rSideJacobian,rightNextStep)

                self._adjustArms(state,s,r,ref,lThetaUpdate,rThetaUpdate)


                leftError = self._error(leftGoal[goal,],'L',state,s)
                rightError = self._error(rightGoal[goal,],'R',state,s)
                # self._simulationDelay(.005,state,s)
                runningCount += 1
            goal += 1

            if goal == 4:
                run = False

        print "\n\n\tFIN\n\n"

    def _adjustArms(self,state,s,r,ref,left,right):

        rSide = self._getCurrentTheta( state , s , 'R' )
        lSide = self._getCurrentTheta(state , s , 'L')



        ref.ref[ha.RSP] = rSide[0] + right[0]
        ref.ref[ha.RSR] = rSide[1] + right[1]
        ref.ref[ha.RSY] = rSide[2] + right[2]
        ref.ref[ha.REB] = rSide[3] + right[3]
        ref.ref[ha.RWY] = rSide[4] + right[4]
        ref.ref[ha.RWP] = rSide[5] + right[5]

        ref.ref[ha.LSP] = lSide[0] + left[0]
        ref.ref[ha.LSR] = lSide[1] + left[1]
        ref.ref[ha.LSY] = lSide[2] + left[2]
        ref.ref[ha.LEB] = lSide[3] + left[3]
        ref.ref[ha.LWY] = lSide[4] + left[4]
        ref.ref[ha.LWP] = lSide[5] + left[5]

        r.put(ref)

    def _raiseArmsToCenter(self,state,s,r,ref):

        rSide = self._getCurrentTheta( state , s , 'R' )
        lSide = self._getCurrentTheta(state , s , 'L')


        # print rSide[0]

        rightSide = np.linspace(rSide[0],-np.pi/2.,10.)
        leftSide = np.linspace(lSide[0],-np.pi/2.,10.)

        moving = True
        inc = 0
        while moving:



            ref.ref[ha.RSP] = rightSide[inc]
            ref.ref[ha.LSP] = leftSide[inc]

            r.put(ref)

            inc += 1

            self._simulationDelay(.1,state,s)


            if inc >= 10:
                moving = False

    def _error(self,goal , side , state , s):

        current = self._getFK(0,False,side,state,s)

        return np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2 + (current[2]-goal[2])**2)

    def _nextStep(self,step,distance,state,s,side,goal):
        current = self._getFK(0,False,side,state,s)
        dx = (goal[0] - current[0]) * step / distance
        dy = (goal[1] - current[1]) * step / distance
        dz = (goal[2] - current[2]) * step / distance

        return np.transpose(np.array([dx,dy,dz]))

    def _getJacobian(self,side,deltaTheta,state,s):
        out = np.zeros([3,np.size(self.rightCoordinateNames)])

        for i in range(np.shape(out)[0]):
            for j in range(np.shape(out)[1]):
                deltaChange = self._getFK(deltaTheta,True,side,state,s,j)

                out[i,j] = deltaChange[i] / deltaTheta

        return out

    def _getFK(self , thetaUpdate , update, side , state , s , j=False):


        if side == 'L':

            currentTheta = self._getCurrentTheta(state,s,'L')

        elif side == 'R':

            currentTheta = self._getCurrentTheta(state , s ,'R')

        else:
            print "error"

        if update == True:

            currentTheta[j] += thetaUpdate

        currentTheta = np.squeeze(np.transpose(currentTheta))



        T0 = np.eye(4)
        T0[1,3] = self.sizes[0]

        T1 = np.eye(4)


        T2 = np.eye(4)


        T3 = np.eye(4)
        T3[2,3] = self.sizes[1]

        T4 = np.eye(4)
        T4[2,3] = self.sizes[2]

        T5 = np.eye(4)

        out = np.dot(self._pitch(currentTheta[0]) , T0)
        out = np.dot(out,np.dot(self._roll(currentTheta[1]),T1))



        out = np.dot(out,np.dot(self._yaw(currentTheta[2]),T2))


        out = np.dot(out,np.dot(self._pitch(currentTheta[3]),T3))
        out = np.dot(out,np.dot(self._yaw(currentTheta[4]),T4))
        out = np.dot(out,np.dot(self._pitch(currentTheta[5]),T5))

        return np.dot(out[0:3,0:3],out[0:3,3])


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
            [np.cos(theta) , -np.sin(theta) , 0. , 0.],
            [np.sin(theta) , np.cos(theta) , 0. , 0.],
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
                    print "UGH ERROR IN RIGHT DH-ARRAY THETA ALLOCATION",self.rightCoordinateNames[i]

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


if __name__ == "__main__":
    jacobianForward()

