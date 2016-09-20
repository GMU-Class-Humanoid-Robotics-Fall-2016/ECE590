#! /usr/bin/env python

"""




"""


import numpy

class rotationMatrix(object):

    def __init__(self, x , y , z, roll , pitch , yaw):




        self.rotation = numpy.dot(numpy.array([ [numpy.cos(pitch)*numpy.cos(yaw) , -numpy.cos(pitch)*numpy.sin(yaw) , numpy.sin(pitch)] ,
                                 [numpy.cos(roll)*numpy.sin(yaw) + numpy.cos(yaw)*numpy.sin(roll)*numpy.sin(pitch) , numpy.cos(roll)*numpy.cos(yaw) - numpy.sin(roll)*numpy.sin(pitch)*numpy.sin(yaw) , -numpy.cos(pitch)*numpy.sin(roll)] , 
                                 [numpy.sin(roll)*numpy.sin(yaw) - numpy.cos(roll)*numpy.cos(yaw)*numpy.sin(pitch) , numpy.cos(yaw)*numpy.sin(roll) + numpy.cos(roll)*numpy.sin(pitch)*numpy.sin(yaw) , numpy.cos(roll)*numpy.cos(pitch)] ]) ,

                             numpy.array([[x],[y],[z]]) )

        self._return()

    def _return(self):

        return self

