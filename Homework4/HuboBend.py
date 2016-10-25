#! /usr/bin/env python

"""






"""

from inverseKinematics2DOF import ik2DOF
import hubo_ach as ha
import ach
import ach_py
import sys
import time
from ctypes import *
import numpy as np

class makeHuboBend(object):

    def __init__(self):

        low = .3
        high = .78285
        increment = 15
        lowerLeg = .30038
        upperLeg = .30003
        torso = .18247

        out = ik2DOF(low, 0., lowerLeg, upperLeg)

        theta1 = out.theta1
        theta2 = out.theta2
        theta3 = 0. - theta1 - theta2
        # print theta1
        # print theta2

        s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
        r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

        state = ha.HUBO_STATE()
        ref = ha.HUBO_REF()

        # sim = ha.HUBO_VIRTUAL()
        [status , framesize] = s.get(state,wait=False,last=True)
        currentValues = self._returnDOFValues( state, s)
        OGTheta1 = currentValues[17]
        OGTheta2 = currentValues[16]
        OGTheta3 = currentValues[15]
        stillGoing = True


        workingInc = 0

        while stillGoing:


            self._sendCommand(s , r , ref , state , theta1 , theta2 , theta3 , increment , 'DOWN')

            self.simulationDelay(1,state,s)

            self._sendCommand(s , r , ref , state , OGTheta1 , OGTheta2 , OGTheta3 , increment , 'UP')

            self.simulationDelay(2, state, s)

            workingInc += 1

            if workingInc > 4:
                stillGoing = False


    def _sendCommand(self, s , r , ref , state , theta1 , theta2 , theta3 , increment , direction):

        currentValues = self._returnDOFValues( state, s)

        if direction == 'DOWN':

            print "GOING DOWN"

            ankleInc = np.linspace(currentValues[17] , theta1 , increment)
            kneeInc = np.linspace(currentValues[16] , theta2 , increment)
            hipInc = np.linspace(currentValues[15], theta3 , increment)

        elif direction == 'UP':

            print "GOING UP"

            ankleInc = np.linspace(currentValues[17] , theta1 , increment)
            kneeInc = np.linspace(currentValues[16] , theta2 , increment)
            hipInc = np.linspace(currentValues[15] , theta3 , increment)


        moving = True
        inc = 0
        while moving:


            ref.ref[ha.RAP] = ankleInc[inc]
            ref.ref[ha.RKN] = kneeInc[inc]
            ref.ref[ha.RHP] = hipInc[inc]
            ref.ref[ha.LAP] = ankleInc[inc]
            ref.ref[ha.LKN] = kneeInc[inc]
            ref.ref[ha.LHP] = hipInc[inc]

            r.put(ref)

            waiting = True

            self.simulationDelay(.1,state,s)
            inc += 1
            if inc >= increment:
                moving = False

    def simulationDelay( self, waitTime , state , s ):
        [statuss, framesizes] = s.get(state, wait=False, last=False)
        current = state.time
        while (True):
            [statuss, framesizes] = s.get(state, wait=True, last=False)

            if ((state.time - current) > waitTime):
                break

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
    makeHuboBend()