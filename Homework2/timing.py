#! /usr/env/bin python

"""






"""

import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
import signal

class stopping(object):

    def __init__(self , hz , overallTime):

        desiredDeltaTime = 1. / hz

        iterationTotal = overallTime / desiredDeltaTime

        iteration = 0

        saveTime = []#np.array([iterationTotal,1],dtype=float)
        curTime = time.time()
        while iteration < iterationTotal:

            # curTime = time.time()
            np.linalg.inv(np.random.random([500,500]))

            sleep = desiredDeltaTime - (time.time() - curTime)

            if np.sign(sleep) == 1:
                time.sleep(desiredDeltaTime - (time.time() - curTime))

            saveTime.append( time.time()-curTime )

            print "Current Delta T = {} , Running Mean Delta T = {}".format( saveTime[iteration] , np.sum(saveTime)/(1+iteration) )

            iteration += 1

            curTime = time.time()

        RMS = np.mean(np.sqrt( np.power( np.squeeze(np.array([saveTime])) - desiredDeltaTime ,2 ) ) )

        print "Finished, Mean Time = {} For A Difference Of {} And RMSE Of {}".format( np.mean(saveTime) , np.mean(saveTime) - desiredDeltaTime , RMS)

        self.saveTime = saveTime

        self._returnThis()

    def _returnThis(self):
        return self

class waiting(object):

    def __init__(self , hz , overallTime):

        desiredDeltaTime = 1. / hz

        iterationTotal = overallTime / desiredDeltaTime

        iteration = 0

        saveTime = []#np.array([iterationTotal,1],dtype=float)

        curTime = time.time()
        while iteration < iterationTotal:


            np.linalg.inv(np.random.random([500,500]))

            timeWaiting = time.time() - curTime

            while timeWaiting < desiredDeltaTime:
                timeWaiting = time.time() - curTime

            saveTime.append( time.time()-curTime )

            print "Current Delta T = {} , Running Mean Delta T = {}".format( saveTime[iteration] , np.sum(saveTime)/(1+iteration) )

            iteration += 1
            curTime = time.time()

        RMS = np.mean(np.sqrt( np.power( np.squeeze(np.array([saveTime])) - desiredDeltaTime ,2 ) ) )

        print "Finished, Mean Time = {} For A Difference Of {} And RMSE Of {}".format( np.mean(saveTime) , np.mean(saveTime) - desiredDeltaTime , RMS)

        self.saveTime = saveTime

        self._returnThis()

    def _returnThis(self):
        return self

class interrupt(object):

    def __init__(self , hz , overallTime):

        desiredDeltaTime = 1. / hz



        iterationTotal = overallTime / desiredDeltaTime

        self.iteration = 0


        self.saveTime = []#np.array([iterationTotal,1],dtype=float)
        self.time = time.time()
        run = True

        signal.signal(signal.SIGALRM, self._runThis)
        signal.setitimer(signal.ITIMER_REAL,desiredDeltaTime,desiredDeltaTime)

        while run == True:
            if self.iteration < iterationTotal:
                pass
            else:
                signal.setitimer(signal.ITIMER_REAL,0)
                run = False

        RMS = np.mean(np.sqrt( np.power( np.squeeze(np.array([self.saveTime])) - desiredDeltaTime ,2 ) ) )

        print "Finished, Mean Time = {} For A Difference Of {} And RMSE Of {}".format( np.mean(self.saveTime) , np.mean(self.saveTime) - desiredDeltaTime , RMS)

        self._returnThis()

    def _runThis(self , signum , _):

        self.saveTime.append(time.time() - self.time)
        print "Current Delta T = {} , Running Mean Delta T = {}".format( self.saveTime[self.iteration] , np.sum(self.saveTime)/(1+self.iteration ))

        self.iteration += 1
        self.time = time.time()

    def _returnThis(self):
        return self



class simulation(object):

    def __init__(self , hz , overallTime):

        rospy.init_node('simulation',anonymous=True)

        rate = rospy.Rate(hz)

        desiredDeltaTime = 1. / hz

        iterationTotal = overallTime / desiredDeltaTime

        iteration = 0

        saveTime = []#np.array([iterationTotal,1],dtype=float)

        while not rospy.is_shutdown():



            startTime = rospy.get_rostime()
            np.linalg.inv(np.random.random([500,500]))
            rate.sleep()
            curTime = rospy.get_rostime()
            saveTime.append((curTime.to_nsec() -startTime.to_nsec())*1e-9)


            print "Current Delta T = {} , Running Mean Delta T = {}".format( saveTime[iteration] , np.sum(saveTime)/(1+iteration) )
            iteration +=1



            if iteration < iterationTotal:
                pass
            else:
                break


        RMS = np.mean(np.sqrt( np.power( np.squeeze(np.array([saveTime])) - desiredDeltaTime ,2 ) ) )

        print "Finished, Mean Time = {} For A Difference Of {} And RMSE Of {}".format( np.mean(saveTime) , np.mean(saveTime) - desiredDeltaTime , RMS)

        self.saveTime = saveTime

        self._returnThis()

    def _returnThis(self):
        return self









if __name__=="__main__":
    time.sleep(10)
    print "\n\n\n" \
          "This program will demonstrate three different ways to wait \n" \
          "for a delta t to pass before going forward.  The program will \n" \
          "run for a total of 10 seconds, printing data at 5 hz.  The \n" \
          "data being printed is included in the desired delta t and is \n" \
          "taken into consideration.  The program calculates the inverse \n" \
          "of a 500x500 random number matrix at each iteration.  "
    time.sleep(5)

    print "\n\n\n" \
          "The first program to run is where the loop is waiting for delta T to \n" \
          "pass before moving forward.  Starting in 5 seconds...\n\n\n"
    time.sleep(5)

    stoppingResults = np.reshape(np.array([stopping(5,10).saveTime]),[50,1])
    print np.shape(stoppingResults)

    print "\n\n\n" \
          "The second program to run is where the a continuous loop is calculated \n" \
          "for delta T to pass before moving forward.  Starting in 5 seconds...\n\n\n"
    time.sleep(5)

    waitingResults = np.reshape(np.array([waiting(5,10).saveTime]),[50,1])


    print "\n\n\n" \
          "The third program to run is where the a loop interrupt is calculated \n" \
          "for delta T to pass before moving forward.  Starting in 5 seconds...\n\n\n"
    time.sleep(5)

    interruptResults = np.reshape(np.array([interrupt(5,10).saveTime]),[50,1])

    print "\n\n\n" \
          "The fourth program to run is where a publishing frequency is determined \n" \
          "for delta T by a simulated clock.  Starting in 5 seconds...\n\n\n"
    time.sleep(5)

    simulatedResults = np.reshape(np.array([simulation(5,10).saveTime]),[50,1])

    xAxis = np.reshape(np.array([np.linspace(1,50,50,endpoint=True)]),[50,1])

    print "\n\n\nA Plot Is Coming... "
    time.sleep(2)
    print "Wait For It... "
    time.sleep(2)
    print "Here It Is... "
    time.sleep(2)
    print "BOOM"
    time.sleep(2)

    plt.figure()

    title_font = {'fontname':'Arial', 'size':'40', 'color':'black', 'weight':'bold',
          'verticalalignment':'bottom'}
    axis_font = {'fontname':'Arial', 'size':'24', 'weight':'bold'}
    plt.title(r"Plot Of Iteration Verses $\Delta$t", **title_font)
    plt.xlabel("Iteration Number",**axis_font)
    plt.ylabel(r"$\Delta$t (Seconds)",**axis_font)

    plt.plot(xAxis, waitingResults, color='blue', linestyle='dashed', marker='o',
             markerfacecolor='blue', markersize=6,label="Wait")
    plt.plot(xAxis, stoppingResults, color='red', linestyle='dashed', marker='o',
             markerfacecolor='red', markersize=6,label="Stop")
    plt.plot(xAxis, simulatedResults, color='green', linestyle='dashed', marker='o',
             markerfacecolor='green', markersize=6,label="Simulate")
    plt.plot(xAxis, interruptResults, color='orange', linestyle='dashed',marker='o',
             markerfacecolor='orange', markersize=6,label="Interrupt")

    # plt.axis([0,51,0,.25])
    plt.legend()
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    plt.show()