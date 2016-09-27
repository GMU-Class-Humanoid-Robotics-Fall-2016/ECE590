#! /usr/env/python

"""





"""

import hubo_ach as ha
import ach
import sys
import time
from ctypes import *
import math


def simSleep(T):
    [statuss, framesizes] = s.get(state, wait=False, last=False)
    tick = state.time
    while (True):
        [statuss, framesizes] = s.get(state, wait=True, last=False)
        print state.time , tick
        if ((state.time - tick) > T):
            break


# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
# s.flush()
# r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state)
[statuss, framesizes] = s.get(state, wait=False, last=False)

run = True

while run:

    simSleep(10)