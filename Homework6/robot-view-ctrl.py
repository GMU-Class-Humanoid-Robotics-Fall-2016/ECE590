#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
from skimage import color
import skimage.morphology as morphology
import math
import matplotlib.pyplot as plt




dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 640
ny = 480

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    img = np.zeros((newx,newy,3), np.uint8)
    c_image = img.copy()
    vid = cv2.resize(c_image,(newx,newy))
    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
        img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        # cv2.imshow("wctrl", img)



        img2 = np.array(img,dtype=float)
        img2 = np.reshape(img,[np.shape(img)[0] * np.shape(img)[1],3])
        img2 = img2/255.

        maxVal = np.max(img2,axis=1)
        maxInd = np.argmax(img2,axis=1)
        minVal = np.min(img2,axis=1)
        minInd = np.argmin(img2,axis=1)
        diff = maxVal - minVal
        hue = np.zeros([np.shape(img2)[0],1])


        ind = np.where(maxVal == minVal)
        if np.size(ind) != 0:
            hue[ind] = np.nan


        ind = np.where(maxInd == 0)
        if np.size(ind) != 0:
            hue[ind,0] = (img2[ind,1] - img2[ind,2])/diff[ind] % 6


        ind = np.where(maxInd == 1)
        if np.size(ind) != 0:
            hue[ind,0] = (img2[ind,2] - img2[ind,0])/diff[ind] + 2


        ind = np.where(maxInd == 2)
        if np.size(ind) != 0:
            hue[ind,0] = (img2[ind,0] - img2[ind,1])/diff[ind] + 4

        hue = hue * np.pi / 3.

        lowerBound = np.where(np.reshape(hue, [np.shape(img)[0] , np.shape(img)[1]]) > 3.66519)

        if np.shape(lowerBound)[1] != 0:
            upCenter = (np.max(lowerBound[0]) + np.min(lowerBound[0])) / 2.
            lowCenter = (np.max(lowerBound[1]) + np.min(lowerBound[1])) /2.
            print 'THE CENTER IS LOCATED AT [{} , {}]'.format(upCenter , lowCenter)
            ind = np.zeros([np.shape(img)[0],np.shape(img)[1]])
            ind[upCenter,lowCenter] = 1
            dilatedImage = morphology.dilation(ind,selem=morphology.disk(10))
            ind = np.where(dilatedImage == 1)
            img[ind[0],ind[1],:]= 255







        cv2.imshow("wctrl", img)


        cv2.waitKey(10)

    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # img        = cv image in BGR format

    ref.ref[0] = -0.5
    ref.ref[1] = 0.5
    #
    # if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
    #
    #     vid2 = cv2.resize(vid,(nx,ny))
    #     img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
    #     cv2.imshow("wctrl", img)
    #
    #     # greyImage = color.rgb2grey(img)
    #     # bwImage = np.reshape(greyImage,[np.size(greyImage),1])
    #     # bwInds = np.where(greyImage < ((-np.std(bwImage) + np.mean(bwImage))))
    #     # bwImage = np.copy(greyImage)
    #     # bwImage[bwInds] = c_uint8(0)
    #     # bwImage[bwImage!= 0.] = c_uint8(255)
    #     #
    #     # disk = morphology.disk(10)
    #     #
    #     # erodedImage = (morphology.erosion(bwImage,selem=disk))
    #     # dilatedImage = morphology.dilation(bwImage,selem=disk)
    #     #
    #     # openedImage = morphology.erosion(morphology.dilation(bwImage,selem=disk),selem=disk)
    #
    #     # redChannel = np.array(img[:,:,0],dtype=np.int8)
    #     # greenChannel = np.array(img[:,:,1],dtype=np.int8)
    #     # blueChannel = np.array(img[:,:,2],dtype=np.int8)
    #
    #     # # print np.max(img[:,:,0])
    #     # hue = color.rgb2hsv(img)
    #     # hue = hue[:,:,1]
    #     # hue = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    #     # print np.max(hue[:,:,0])
    #     #
    #     #
    #     #### Find Red Ball ####
    #
    #     # M = np.argmax(img,axis=2)
    #     # m = np.argmin(img,axis=2)
    #     # C = M-m
    #     #
    #     # hue = np.zeros([np.shape(redChannel)[0],np.shape(redChannel)[1]])
    #     #
    #     # for i in range(np.shape(M)[0]):
    #     #     for j in range(np.shape(M)[1]):
    #     #
    #     #         if C[i,j] == 0.:
    #     #             hue[i,j] = np.nan
    #     #         elif M[i,j] == 0:
    #     #             hue[i,j] = np.fmod((img[i,j,1] - img[i,j,2]) / C[i,j],6.) * 60
    #     #         elif M[i,j] == 1:
    #     #             hue[i,j] = ((img[i,j,2] - img[i,j,0]) / C[i,j] + 2.) * 60.
    #     #         elif M[i,j] == 2:
    #     #             hue[i,j] = ((img[i,j,0] - img[i,j,1] + 4) /C[i,j]) * 60.
    #     #
    #     #
    #     # redInds = np.hstack([np.where(hue < 30.) , np.where(hue > 330.)])
    #     #
    #     # print redInds
    #
    #
    #     img = np.array(img,dtype=float)
    #
    #     print np.max(img[:,:,0])
    #
    #
    #     # hsvImg = cv2.cvtColor(vid2,cv2.COLOR_BGR2HSV_FULL)
    #     # print np.max(np.max(hsvImg[:,:,0],axis=1))
    #     #
    #     #
    #     #
    #
    #
    #
    #     # cv2.imshow("Grey_Image", greyImage)
    #     # cv2.imshow("Black_White_Image", bwImage)
    #     # cv2.imshow("Eroded_Image",erodedImage)
    #     # cv2.imshow("Dilated_Image",dilatedImage)
    #     # cv2.imshow("Opened_Image",openedImage)
    #     # cv2.imshow("Red_Color_Channel",redChannel)
    #     # cv2.imshow("Green_Color_Channel",greenChannel)
    #     # cv2.imshow("Blue_Color_Channel",blueChannel)
    #     #
    #     #
    #
    #
    #
    #
    #
    #
    #
    #
    #     cv2.waitKey(10)
    # else:
    #     raise ach.AchException( v.result_string(status) )




    print 'Sim Time = ', tim.sim[0]
    
    # Sets reference to robot
    r.put(ref)

    # Sleeps
    time.sleep(0.1)   
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
