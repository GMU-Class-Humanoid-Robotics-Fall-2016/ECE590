#! /usr/bin/env python

"""

Angle Constraints Are From:
    http://www.golems.org/papers/OFlaherty13-hubo-kinematics-techreport.pdf



"""


import hubo_ach as ha
import ach
import ach_py
import sys
import time
from ctypes import *
import numpy as np
import openhubo



class huboWalk(object):

    def __init__(self , constraintsAngles , numberOfSteps , accuracy = 0.01):

        self.initTime = 0.0



        bendKneesNames = np.array(['RKN' ,  'LKN' , 'RHP' , 'LHP' , 'RAP' , 'LAP' , 'RHR' , 'LHR' , 'RAR' , 'LAR'])
        bendKneesValues = np.array([1.3 ,   1.3 ,   -.83,    -.83 ,   -0.6 ,  -0.6 , 0. , 0. , 0. ,  0.],dtype=float)

        straightKneesNames = np.array(['RKN' ,  'LKN' , 'RHP' , 'LHP' , 'RAP' , 'LAP' , 'RHR' , 'LHR' , 'RAR' , 'LAR'])
        straightKneesValues = np.array([0. ,   0. ,   0.,    0. ,   0. , 0. ],dtype=float)



        shiftWeightNames = np.array(['RKN' ,  'LKN' , 'RHP' , 'LHP' , 'RAP' , 'LAP' ,'RHR' , 'LHR' , 'RAR' , 'LAR'])
        shiftWeightLeftValues = np.array([1.3 ,   1.3 ,   -.83,    -.83 ,   -0.6 ,  -0.6 ,-0.28 , -0.28 , 0.28 , 0.28],dtype=float)
        shiftWeightCenterValues = np.array([0.,0.,0.,0.],dtype=float)
        shiftWeightRightValues = np.array([1.3 ,   1.3 ,   -.83,    -.83 ,   -0.6 ,  -0.6 ,.28 , 0.28 , -0.28 , -0.28],dtype=float)

        kneePunchNames =  np.array(['RKN' ,  'LKN' , 'RHP' , 'LHP' , 'RAP' , 'LAP' , 'RHR' , 'LHR' , 'RAR' , 'LAR'])
        kneePunchRightValues = np.array([1.4 , 1.3 , -1. , -.83 , -0.6 , -0.6 , -.28 , -.28 , .28 , .28],dtype=float)
        kneePunchLeftValues = np.array([1.3 , 1.4 , -.83 , -.9 , -0.6 , -0.6 , .28 , .28 , -.28 , -.28],dtype=float)


        landLegNames = np.array(['RKN' ,  'LKN' , 'RHP' , 'LHP' , 'RAP' , 'LAP' , 'RHR' , 'LHR' , 'RAR' , 'LAR'])
        landRightLegValues = np.array([1.3017 , 1.3 , -.269 , -0.83 ,0.,-0.6,-.28,-.28,0,0.])
        landLeftLegValues = np.array([1.3 , 1.3017 , -0.83 , -.269 , -0.6 , 0.,.28,.28, 0.,.0])

        shiftWeightAgainNames = np.array(['RKN' ,  'LKN' , 'RHP' , 'LHP' , 'RAP' , 'LAP' , 'RHR' , 'LHR' , 'RAR' , 'LAR'])
        shiftWeightLeftToRightValues = np.array([1.3017 , 1.899 , -.269 , -.6225 , 0. , 0. , 0. , 0. , 0. , 0. ],dtype=float)
        shiftWeightRightToLeftValues = np.array([1.3017 , 1.899 , -.269 , -.6225 , 0. , 0. , 0. , 0. , 0. , 0. ],dtype=float)



        s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
        r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

        state = ha.HUBO_STATE()
        ref = ha.HUBO_REF()

        sim = ha.HUBO_VIRTUAL()
        [status , framesize] = s.get(state,wait=False,last=True)
        current = np.zeros([np.size(shiftWeightNames),1])
        gain = 0
        print 'bend'
        current = self._sendCommand(r , ref , bendKneesNames , bendKneesValues,current , gain , correct=False)
        print 'shift'
        current = self._sendCommand(r , ref , shiftWeightNames , shiftWeightLeftValues,current , gain , correct=True)
        print 'kneePunch'
        current = self._sendCommand(r , ref , kneePunchNames , kneePunchRightValues,current , gain , correct=True)
        current = self._sendCommand(r,  ref, landLegNames, landRightLegValues, current, gain, correct=True)
        current = self._sendCommand(r , ref, shiftWeightAgainNames, shiftWeightLeftToRightValues, current, gain, correct=True)

        simTime = ha.HUBO_LOOP_PERIOD




    def _sendCommand(self , r , ref , desiredName , desiredValue , current , gain , correct ,acceptableError=0.001 , iteration=25.):


        if correct == True:

            changeThis = np.zeros([np.size(current),1])

            for i in range(np.size(desiredValue)):
                if desiredValue[i] > current[i]:
                    x = [np.squeeze(desiredValue[i]),np.squeeze(current[i])]
                    changeThis[i] = np.abs(np.diff(x,axis=0))
                elif desiredValue[i] < current[i]:
                    x = [np.squeeze(desiredValue[i]),np.squeeze(current[i])]
                    changeThis[i] = -1 * np.abs(np.diff(x,axis=0))
                    # changeThis[i] = np.abs(current[i]) - np.abs(desiredValue[i])  * (-1)
                else:
                    changeThis[i] = 0
        else:
            changeThis = np.array([desiredValue])



        continueThis = True
        curIter = 0

        changeThis = np.transpose(changeThis)

        while curIter < iteration:


            addThis = current + changeThis * (1./iteration) * float(curIter)

            addThis = np.squeeze(addThis)
            for i in range(np.size(desiredName)):
                if desiredName[i] == 'RKN':
                    ref.ref[ha.RKN] = addThis[i]
                elif desiredName[i] == 'LKN':
                    ref.ref[ha.LKN] = addThis[i]
                elif desiredName[i] == 'RHP':
                    ref.ref[ha.RHP] = addThis[i]
                elif desiredName[i] == 'LHP':
                    ref.ref[ha.LHP] = addThis[i]
                elif desiredName[i] == 'RAP':
                    ref.ref[ha.RAP] = addThis[i]
                elif desiredName[i] == 'LAP':
                    ref.ref[ha.LAP] = addThis[i]
                elif desiredName[i] == 'RHR':
                    ref.ref[ha.RHR] = addThis[i]
                elif desiredName[i] == 'LHR':
                    ref.ref[ha.LHR] = addThis[i]

            r.put(ref)
            time.sleep(.5)
            curIter += 1


        return addThis




    def _checkForCloseness(self,desired,state,acceptableError = .001):

        correct = np.ones([np.shape(desired)[0],2])
        dof = self._returnDOFValues(state)

        print dof

        for i in range(np.shape(desired)[0]):

            correct[i,1] = np.squeeze(np.array([dof[np.squeeze(np.array([np.where(dof[:,0] == desired[i,0])],int)),1]],float))

            if (correct[i,1] - np.squeeze(np.array([desired[i,1]],float))) < acceptableError:
                correct[i,0] = 0

            else:

                if np.sign(correct[i,1]) == np.sign(np.squeeze(np.array([desired[i,1]],float))) and np.sign(np.squeeze(np.array([desired[i,1]],float))) == -1:
                    correct[i,0] = -1

                elif np.sign(correct[i,1]) != np.sign(np.squeeze(np.array([desired[i,1]],float))) and np.sign(np.squeeze(np.array([desired[i,1]],float))) == -1:
                    correct[i,0] = -1

        return correct

    def raiseLeg(self , move , stay):

        print 'raise leg'

    def straightLeg(self , move , stay):

        print 'straight leg'

    def _returnDOFValues(self , state ):

        return np.array([
        ['RSP' , state.joint[ha.RSP].pos],
        ['RSR' ,state.joint[ha.RSR].pos],
        ['RSY' ,state.joint[ha.RSY].pos],
        ['REB' ,state.joint[ha.REB].pos],
        ['RWY' ,state.joint[ha.RWY].pos],
        ['RWP' ,state.joint[ha.RWP].pos],
        ['LSP' ,state.joint[ha.LSP].pos],
        ['LSR' ,state.joint[ha.LSR].pos],
        ['LSY' ,state.joint[ha.LSY].pos],
        ['LEB' ,state.joint[ha.LEB].pos],
        ['LWY' ,state.joint[ha.LWY].pos],
        ['LWP' ,state.joint[ha.LWP].pos],
        ['WST' ,state.joint[ha.WST].pos],
        ['RHY' ,state.joint[ha.RHY].pos],
        ['RHR' ,state.joint[ha.RHR].pos],
        ['RHP' ,state.joint[ha.RHP].pos],
        ['RKN' ,state.joint[ha.RKN].pos],
        ['RAP' ,state.joint[ha.RAP].pos],
        ['RAR' ,state.joint[ha.RAR].pos],
        ['LHY' ,state.joint[ha.LHY].pos],
        ['LHR' ,state.joint[ha.LHR].pos],
        ['LHP' ,state.joint[ha.LHP].pos],
        ['LKN' ,state.joint[ha.LKN].pos],
        ['LAP' ,state.joint[ha.LAP].pos],
        ['LAR' ,state.joint[ha.LAR].pos]
        ])

    def PID(self , Kp , Ki , Kd , desired , current , error , dt):
        curError = desired - current
        kp = Kp * curError
        ki = Ki * (error + curError)
        kd = Kd * curError / dt

        return curError , kp + ki + kd


    def _delayThis(self,prevTime,frequency):

        actualTime = time.time()-self.t_last
        time.sleep(actualTime * ())


        # virtualHuboLog('Sim time: {:.3f}, Actual time: {:.3f}, RT rate: {:.3f}% T= {:.6f}'.format(ideal_time,actual_time,ideal_time/actual_time*100,openhubo.TIMESTEP))  #
        #virtualHuboLog('Sim time: {:.3f}, Actual time: {:.3f}, RT rate: {:.3f}%'.format(ideal_time,actual_time,ideal_time/actual_time*100))
        self.t_last=t
        self.count=0
        return time.time()


if __name__=="__main__":

    constraintsAngles = np.array([
        ['RHR' , -0.6 , 0.],
        ['RHP' , -1.3 , 1.4],
        ['RKN' , 0. , 2.5],
        ['RAP' , -1.3 , 1.8],
        ['RAR' , -0.2 , 0.3],
        ['LHR' , 0. , -0.6],
        ['LHP' , -1.3 , 1.4],
        ['LKN' , 0. , 2.5],
        ['LAP' , -1.3 , 1.8],
        ['LAR' , -0.3 , 0.2]

    ])

    huboWalk(constraintsAngles , 0 , accuracy = 0.01)
