#! /usr/bin/env python


"""






"""

import numpy as np

def _getFK(theta , l1 , l2):
    X = np.array([[0],[0]])
    X[0] = l1 * np.cos(theta[0]) + l2 * np.cos(theta[0] + theta[1])
    X[1] = l1 * np.sin(theta[0]) + l2 * np.sin(theta[0] + theta[1])

    return X

def _getJ(theta , dtheta , l1 , l2):

    J = np.zeros([2,2])
    J[0,0] = _getFK(theta + np.array([[dtheta] , [0]]) , l1 , l2)[0] / dtheta
    J[0,1] = _getFK(theta + np.array([[0] , [dtheta]]) , l1 , l2)[0] / dtheta
    J[1,0] = _getFK(theta + np.array([[dtheta] , [0]]) , l1 , l2)[1] / dtheta
    J[1,1] = _getFK(theta + np.array([[0] , [dtheta]]) , l1 , l2)[1] / dtheta

    return J

def _getMet(current , goal):

    return np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)

def _getNext(current , goal , de):

    h = _getMet(current , goal)
    dx = (goal[0] - current[0]) * (de/h)
    dy = (goal[1] - current[1]) * (de/h)

    return np.array([[dx],[dy]])


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



