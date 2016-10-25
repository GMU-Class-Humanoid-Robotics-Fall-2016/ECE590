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




    def _getFK(self , theta , side):

        if side == 'L':

            print 'left'

        elif side == 'R':

            print 'right'

        else:
            print "error"




    def _roll(self,theta):

        return np.array([
            [0. , 0. , 0.],
            [0. , np.cos(theta) , -np.sin(theta)],
            [0. , np.sin(theta) , np.cos(theta)]
        ])



    def _pitch(self,theta):

        return np.array([
            [np.cos(theta) , 0. , np.sin(theta)],
            [0. , 0. , 0.],
            [-np.sin(theta) , 0. , np.cos(theta)]
        ])

    def _yaw(self,theta):

        return np.array([
            [np.cos(theta) , -np.sin(theta) , 0.],
            [np.sin(theta) , np.cos(theta) , 0.],
            [0. , 0. , 0.]
        ])










