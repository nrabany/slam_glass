from __future__ import print_function
import roslib 
import rospkg
import numpy as np
from scipy import stats
import rospy
import math
import time
import tf
import copy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from pyflann import *
from detect_peaks import detect_peaks
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.externals import joblib
import scipy.ndimage.filters as fi
from matplotlib.colors import LinearSegmentedColormap

def _plotHoughLines(senRange, accumulator, alphas, thetas, rhos, gridSize, maxRange):
    """
     For debuging
     Visualize several middle result of Hough Transform line detection"""

    fig1 = plt.figure()

    # 2D figure of accumulator
    ax0 = fig1.add_subplot(121)
    ax0.imshow(accumulator, cmap='jet')
    ax0.axis('tight')
    ax0.set_title('Accumulator of Hough Transform')
    ax0.set_xlabel('Theta')
    ax0.set_ylabel('Rho')
    ax0.grid(which='both', color='r')
    ax0.set_xticks(np.arange(np.rad2deg(thetas[0]), np.rad2deg(thetas[-1]),10))
    ax0.set_yticks(np.arange(0,2*maxRange/gridSize,20))
    ax0.set_yticklabels(np.arange(-maxRange,maxRange, gridSize*20))

    # polar figure of raw LRF scan data
    ax2 = fig1.add_subplot(122, projection = 'polar')
    ax2.plot(alphas, senRange, '.b')
    ax2.set_title('LRF Scan Raw Data')

    plt.show()
    
    fig2 = plt.figure()
    # 3D figure of accumulator
    ax1 = fig2.add_subplot(111, projection="3d")
    X, Y = np.meshgrid(np.rad2deg(thetas), rhos)
    ax1.plot_surface(X, Y, accumulator, cmap="coolwarm")
    print("np.average(accumulator) ", np.average(accumulator))
    
    # votes of each anlge after filtering small noise
    accumulator[accumulator<15]=0 # noise threshold 15
    angleVote = np.average(accumulator, axis=0)
    plt.figure()
    plt.plot(np.arange(0,180,1), angleVote)
    print(np.average(accumulator))

def calIncidentAngle(senRange, alphas, gridSize, maxRange, minErrorThr = 0.3, showH=False, showL=False, debug=False):
    """
    Use Hough Transform to detect lines from LRF raw scan data,
    then calculate incident angles of each beams in this scan

    Parameters
    ----------
    senRange: 1-d array, shape (<1081,), in meter
                beam ranges shorter than maxRange; multiple beams, one scan;
    alphas:   1-d array, shape (<1081,), in rad
                beam angles of senRange, from 0 deg to 270 deg
    minErrorThr: scalar in meter
                threshold of error of endpoints to the nearest lines found by Hough Transform
                incident anlge value of points whose error is higher than this threshold will be set to -1 (fail)
    showH:    bool
                plot the results of Hough Transform, function: _plotHoughLines
    showL:    bool
                plot the LRF raw data, detected lines, and surface normal

    Return
    ------
    incidAngle: 1-d array, shape (<1081,), in rad
                from 0 deg to 90 deg, or -1 means cannot detected successfully
                incident angles of all input beams; multiple beams, one scan;	
    """

    angleStep = 1
    thetas = np.deg2rad(np.arange(0, 180, angleStep)) # line angles (180,)
    rhos = np.linspace(-maxRange, maxRange, 2*maxRange/gridSize+1) # line rhos (601,)

    # cache some resuable values
    cos_a = np.cos(alphas)
    sin_a = np.sin(alphas)
    cos_t = np.cos(thetas)
    sin_t = np.sin(thetas)
    num_thetas = len(thetas)

    # Hough accumulator array of theta vs rho
    accumulator = np.zeros((len(rhos), num_thetas), dtype=np.uint64) # shape(601, 180)

    y_coord = senRange*sin_a # y,x in robot local coordinate
    x_coord = senRange*cos_a

    # vote in Hough accumulator
    for i in range(len(alphas)):
        y = y_coord[i]
        x = x_coord[i]
        for t in range(num_thetas):
            rho = x * cos_t[t] + y * sin_t[t]
            t_idx = t
            r_idx = int(round((rho + maxRange) / gridSize))
            accumulator[r_idx, t_idx] +=1

    if showH:   
        _plotHoughLines(senRange, accumulator, alphas, thetas, rhos, gridSize, maxRange)

    # find peak angles in accumulator
    accumulator[accumulator<20]=0  # filter threshold (super parameter)
    angleAccum = np.average(accumulator, axis=0) 

    # fiding peak angles -- add [0,90] with [91, 180] and count itand 2 neighbors
    neiborCount = [np.average(angleAccum[[0,1,89,90,91]])] # for 0 deg
    neiborCount = neiborCount+[(np.sum(angleAccum[i-1:i+2])+ np.sum(angleAccum[i+90-1:i+90+2]))/6.0 
                    for i in range(1,89)] # for 1 to 88 deg 
    neiborCount.append(np.average(angleAccum[[88,89,90,178,179]])) # for 89 deg (0deg = 90deg)

    # find peak angles, can handle two sets of mutually vertical lines
    neiborCount = np.asarray(neiborCount)
    theta1_deg = np.argmax(neiborCount) 
    theta2_deg = theta1_deg + 45 if theta1_deg < 45 else theta1_deg-45
    theta3_deg = theta1_deg + 90
    theta4_deg = theta2_deg + 90

    # find the peak rhos for each angle (also the line function) based on the accumulator
    def _findLine(theta_deg):
        theta = np.deg2rad(theta_deg)
        peakIndex = detect_peaks(accumulator[:, theta_deg], mpd=1, threshold=2, show=False)
        peakRhos = peakIndex*gridSize-maxRange
        if debug: print("theta(deg): ", theta_deg, "\t rhos(m)", peakRhos)

        return theta, peakRhos

    theta1, rhos1 = _findLine(theta1_deg)
    theta2, rhos2 = _findLine(theta2_deg)
    theta3, rhos3 = _findLine(theta3_deg)
    theta4, rhos4 = _findLine(theta4_deg)

    # calculate incident angles of each beam
    copy1 = np.repeat(theta1, len(rhos1))  	# shape(i,)  i, j: peak rho number
    copy2 = np.repeat(theta2, len(rhos2))  	# shape(j,)
    copy3 = np.repeat(theta3, len(rhos3))
    copy4 = np.repeat(theta4, len(rhos4))
    copyTheta = np.concatenate((copy1, copy2, copy3, copy4), axis=0) 	# shape (i+j,)
    rhos = np.concatenate((rhos1, rhos2, rhos3, rhos4), axis=0) 		# shape (i+j,)
    errors = abs(rhos.reshape(1,-1)-senRange.reshape(-1,1)*
                    np.cos(copyTheta.reshape(1,-1)-alphas.reshape(-1,1))) # shape (<1081, i+j)
    try:
        argmin = np.argmin(errors, axis=1) 		# shape(<1081,)
    except:
        print("Error in np.argmin(errors, axis=1). /n errors = ", errors, "/n rhos = ", rhos)
        return None
        
    # find each beam's rhos and theta corresponding to min error 
    minRhos = rhos[argmin]          # shape(<1081,)
    minThetas = copyTheta[argmin]   # shape(<1081,)
    
    # if corresponding Rhos < 0, the real angle from x-axis should be theta+pi
    minThetas[minRhos<0] = minThetas[minRhos<0] + np.pi     # shape(<1081,)
    minErrors = np.amin(errors, axis=1)
    
    # calculate incidentangle 
    incidAngle = abs(minThetas-alphas)
    # correct incident angle calculation error when (alpha<90 and theta>270) in deg
    incidAngle[(alphas < np.pi/2) & (minThetas > np.pi*3/2)] = 2*np.pi - incidAngle[(alphas < np.pi/2) & (minThetas > np.pi*3/2)]
    # set incidAngle of those with high error to the nearest line to -1, meaning failing
    incidAngle[minErrors>minErrorThr] = -1
    
#        # correct incident angle to 90 deg when it < 92 deg (Do or Not?)
#        incidAngle[(np.pi/2 < incidAngle) & (incidAngle < np.deg2rad(92))] = np.pi/2  

    # for debug
    if debug:
        if np.any(incidAngle>np.pi/2):
            print("incidAngle", np.rad2deg(incidAngle[incidAngle>np.pi/2]), "\nalphas", np.rad2deg(alphas[incidAngle>np.pi/2]), 
                    "\nminThetas ", np.rad2deg(minThetas[incidAngle>np.pi/2]), "\nrhos ", minRhos[incidAngle>np.pi/2],
                    "\nminError ", minErrors[incidAngle>np.pi/2], "\naverage minError ", np.average(minErrors))
#            _plotHoughLines(senRange, accumulator, alphas, thetas, rhos, gridSize, maxRange)
        print("theta1_deg's count ", neiborCount[theta1_deg])
        print("theta2_deg's count ", neiborCount[theta2_deg])

    if showL:
        # plot LRF scan and incident angle data in animation
        if "ax2" not in locals():
            fig2 = plt.figure('111')#   
            ax2 = fig2.add_subplot(111, projection = 'polar')

        # plot LRF raw scan data 
        ax2.cla()
        ax2.plot(alphas, senRange, '.b')
        ax2.set_title('LRF Scan Data, Extracted Lines and Surface Normal', y=1.08)
        ax2.set_ylim(0, maxRange) 

        # plot lines represented by each (theta, rho)
        for i in range(len(rhos)):
            alpha = np.arccos(abs(rhos[i])/(maxRange))#-np.pi/10
            indAngle = np.arange(copyTheta[i]-alpha,copyTheta[i]+alpha,np.pi/180.0)

            if rhos[i]>0:
                ax2.plot(indAngle, rhos[i]/np.cos(indAngle-copyTheta[i]))
            else:
                ax2.plot(indAngle+np.pi, -rhos[i]/np.cos(indAngle-copyTheta[i]))

        # plot surface normal arrows
        for k in range(len(alphas)):
            a=alphas[k]
            r=senRange[k]
            t=minThetas[k]
            l=1
            tail=(a,r)
            head=np.array([r*np.cos(a)-l*np.cos(t), r*np.sin(a)-l*np.sin(t)])/(2*maxRange)+0.5
            if (incidAngle[k] < np.pi/2) and k%10==0:
                ax2.annotate('', xy=head, xycoords="axes fraction", xytext=tail, textcoords="data",
                            arrowprops=dict(arrowstyle="->", color="green"),)
            if incidAngle[k] >= np.pi/2:
                ax2.annotate('', xy=head, xycoords="axes fraction", xytext=tail, textcoords="data",
                            arrowprops=dict(arrowstyle="->", color="red"),)
        plt.pause(0.1)

    return incidAngle