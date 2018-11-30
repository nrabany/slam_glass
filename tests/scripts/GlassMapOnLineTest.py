#!/usr/bin/python
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
import yaml



VERBOSE = True

allGridInfo = np.array([]).reshape(0,6)
#n = 0 

class GlassMapBuilder():

    def __init__(self, mapTopic="map", mapMetaTopic="map_metadata", LRFTopic="scan", maxRange = 29):   # LRFTopic = scan because need to remove noise caused by computer lid
        '''Initializa ros publisher, ros subscriber'''
        
        # assign info source topics
        self.mapTopic = mapTopic
        self.LRFTopic = LRFTopic
        self.mapMetaTopic = mapMetaTopic

        # initialize variables
        self.mapMetaData = None # Can be removed
        self.mapHeight = None
        self.mapWidth = None
        self.gridSize= None
        
        self.gridMap = None 
        self.glassMap = None 
        self.finalGlassMap = None
        self.currentScan = None  # show current LRF scan in grid map by matplotlib (might delete later)
        self.occuThreshold = 45 # 0  100
        
        # neural network
        self.GlassNN =joblib.load("/home/nicolas/catkin_ws/src/tests/pkls/NN3.pkl")
        self.Scaler=joblib.load("/home/nicolas/catkin_ws/src/tests/pkls/Scaler3.pkl")
                
        self.senRange = None
        self.senIntensity = None
        self.senMinAng = None
        self.senMaxAng = None
        self.senIncrement = None
        self.robotPose = None
        self.maxRange = maxRange # beams exceed this will be ignored when building glass map
        
        # about plot glass and grid map
        self.frame = 0
        self.axGlass = None
        self.axGrid = None
        self.mapRange = np.array([2000,2000,2000,2000])  # [minX, maxX, minY, maxY] for only showing active map area

        # initialize subscribers
        self.listener()
        self.tfListener = tf.TransformListener()
        rospy.sleep(1)  # Give tf some time to fill its buffer

        # others
        self.flann = FLANN()

            
    def callbackMap(self, Map):
    	"""Callback function of topic /map, get occupancy info of grid map"""
    	
        # to be changed: get the grid number from map_mateData
        # shape (n-grids,) -> (4000,4000) and flip because the arragement of ros /map data
        if self.mapHeight is not None:
            self.gridMap = np.flip(np.asarray(Map.data).reshape(self.mapHeight,-1), axis=0)  # flip to imshow the map, might need to flip back when publish
        if VERBOSE:
			print("Inside callbackMap: get grid map data from Gmapping, map shape: ", self.gridMap.shape, "Map time: ", Map.header.stamp)

    def callbackMapMeta(self, mapMeta):
		"""Callback function of topic /map_metadata, get map data resolution and shape info"""
		self.mapMetaData = mapMeta # Can be removed
		self.mapWidth = mapMeta.width
		self.mapHeight = mapMeta.height
		self.gridSize = round(mapMeta.resolution,2)

		if self.glassMap is None:
			self.glassMap = np.full((self.mapHeight,self.mapWidth), 0.5)
			self.currentScan = np.full((self.mapHeight,self.mapWidth), 0.5)
			self.glassMap[2000,2000] = 1.0	# For balancing the map color when using matplotlib "coolwarm" cmap
			self.glassMap[2000,2001] = 0.0  # but it doesn't seem work, delete it later if confirmed
			print("Inside callbackMapMeta: initialize glassMap, shape: ", self.glassMap.shape, " gridSize is ", self.gridSize)
		if VERBOSE:
			print("Inside callbackMapMeta: get map meta data")

    def callbackLRF(self, laserScan):
    	"""Callback function of topic /first, get LRF range and intensity info"""
    	
        self.senRange = np.asarray(laserScan.ranges)
        self.senIntensity = np.asarray(laserScan.intensities)
        self.senMinAng = laserScan.angle_min
        self.senMaxAng = laserScan.angle_max
        self.senIncrement = laserScan.angle_increment
    #	Debug: comment out probably because rate of LRF is too high, cannot see other info
    # 		if VERBOSE:
    # 			print("Get LRF info. LRF range shape: ", self.senRange.shape, 
    # 					" LRF intensity shape: ", self.senIntensity.shape, "LRF time: ", laserScan.header.stamp)

    def listener(self):
    	"""Initialize 3 subscribers""" 
    	rospy.Subscriber(self.mapMetaTopic, MapMetaData, self.callbackMapMeta)
        rospy.Subscriber(self.mapTopic, OccupancyGrid, self.callbackMap) # already includes map_meta data
        rospy.Subscriber(self.LRFTopic, LaserScan, self.callbackLRF)
       
        if VERBOSE:
            print("Initialize subscribers to topics /%s, /%s and /%s" %(self.mapMetaTopic, self.mapTopic, self.LRFTopic))
    
    def publisher(self):
        """publish glassMap to ROS"""
        # remember to flip the glassMap

    def getRobotPose(self, VERBOSE=False):
        """Get robot pose using /tf """
        now = rospy.get_rostime()+rospy.Duration(0.2)
        self.tfListener.waitForTransform("map", "/laser", rospy.Time(0), rospy.Duration(1.0))
        tfPose = self.tfListener.lookupTransform("map", "/laser", rospy.Time(0))
        
        self.robotPose = np.asarray(tfPose[0])
        robotOrientation = tf.transformations.euler_from_quaternion(tfPose[1])
        self.robotPose[2] = robotOrientation[2]
        if VERBOSE:
            print("inside getRobotPose: get robot pose: ", self.robotPose)

    def calIncidentAngle(self, senRange, alphas, minErrorThr = 0.3, showH=False, showL=False, debug=False):
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
        print(self.gridSize, ', ', self.maxRange)
        angleStep = 1
        thetas = np.deg2rad(np.arange(0, 180, angleStep)) # line angles (180,)
        rhos = np.linspace(-self.maxRange, self.maxRange, 2*self.maxRange/self.gridSize+1) # line rhos (601,)

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
            rho_list = np.array([])
            for t in range(num_thetas):
                rho = x * cos_t[t] + y * sin_t[t]
                t_idx = t
                r_idx = int(round((rho + self.maxRange) / self.gridSize))
                accumulator[r_idx, t_idx] +=1

        if showH:   
            _plotHoughLines(senRange, accumulator, alphas, thetas, rhos, self.gridSize, self.maxRange)

        # find peak angles in accumulator
        accumulator[accumulator<20]=0  # filter threshold (super parameter)
        angleAccum = np.average(accumulator, axis=0) 

        # fiding peak angles -- add [0,90] with [91, 180] and count itself and 2 neighbors
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
            peakRhos = peakIndex*self.gridSize-self.maxRange
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
    #            _plotHoughLines(senRange, accumulator, alphas, thetas, rhos, self.gridSize, self.maxRange)
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
            ax2.set_ylim(0, self.maxRange) 

            # plot lines represented by each (theta, rho)
            for i in range(len(rhos)):
                alpha = np.arccos(abs(rhos[i])/(self.maxRange))#-np.pi/10
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
                head=np.array([r*np.cos(a)-l*np.cos(t), r*np.sin(a)-l*np.sin(t)])/(2*self.maxRange)+0.5
                if (incidAngle[k] < np.pi/2) and k%10==0:
                    ax2.annotate('', xy=head, xycoords="axes fraction", xytext=tail, textcoords="data",
                                arrowprops=dict(arrowstyle="->", color="green"),)
                if incidAngle[k] >= np.pi/2:
                    ax2.annotate('', xy=head, xycoords="axes fraction", xytext=tail, textcoords="data",
                                arrowprops=dict(arrowstyle="->", color="red"),)
            plt.pause(0.1)

        return incidAngle

    def calCoordinate(self, robotPose, senRange, theta):
        """calculate the coordinate of beam endpoint in the grid map
            Inputs:
            robot pose: 3 scalar, (x, y, yaw_deg)
            theta: array.shape (<1081,), valid beams' angles in rad
            senRange: array.shape (<1081,) , valid beams' range """

        yawLocal = theta -np.deg2rad(135) # rad, array.shape (<1081,)
      	yawGlobal = robotPose[2]+yawLocal # rad, array.shape (<1081,)
        x = robotPose[0] + senRange*np.cos(yawGlobal) # array.shape (<1081,)
        y = robotPose[1] + senRange*np.sin(yawGlobal) # array.shape (<1081,)
        
        return np.array((x, y)).T # return sensor coord arrary.shape (<1081, 2)


        
    def oneDIndexToCoord(self, index):
        """
        Unused function
        
        Input the index a one dimension array, output the corresponding coordinate in the map
        """
        width = self.mapMetaData.width
        height = self.mapMetaData.height
        resolution = self.mapMetaData.resolution
        originX = self.mapMetaData.origin.position.x
        originY = self.mapMetaData.origin.position.y

        Y = index/width*resolution + originY
        X = index%width*resolution + originX
        
        return (X, Y)
        
    def classifier(self, senRange, incidAngle, senIntensity):
        """
        Glass or non-glass classifier, default is neural network, can also be SVM
        """
        # input [range, angle, intensity] into neural network
        # rad2deg first because the NN is trained in deg, but here angle is all in rad
        incidAngle = np.rad2deg(incidAngle)
        scanInfo = np.c_[senRange, incidAngle, senIntensity]

        scanInfo = self.Scaler.transform(scanInfo)
        probs = self.GlassNN.predict_proba(scanInfo) # probs.shape (<1081,2) (nonGlassProb, glassProb)
        glassProb = probs[:,1] # glassProb.shape (<1081,)
        return glassProb
    
    
    def updateProb(self, prior, evidence):
        """update the glassMap based on info of each scan based on Bayes rules (just get average before)"""
        newProb = prior * evidence / (prior*evidence + (1-prior)*(1-evidence))
        return newProb
    
    def plotMap(self, glassMap=True, gridMap=False):
        
        minX = self.mapRange[0] - 50
        maxX = self.mapRange[1] + 50
        minY = self.mapRange[2] - 50
        maxY = self.mapRange[3] + 50
        
        if glassMap:
            # Plot glass map
            if self.frame == 1:
                figGlass, self.axGlass = plt.subplots(1)		# Plot for glass map    
            self.axGlass.cla()
            self.axGlass.imshow(self.glassMap[minX:maxX, minY:maxY], interpolation="nearest", cmap="coolwarm")
            self.axGlass.set_title("Glass Map Frame {}".format(self.frame))
        
        if gridMap:
            # Plot grid map
            if self.frame == 1:
                figGrid, self.axGrid = plt.subplots(1)		# Plot for glass map    
            self.axGrid.cla()
            self.axGrid.imshow(self.gridMap[minX:maxX, minY:maxY], interpolation="nearest", cmap="gray",alpha = 0.5)
            self.axGrid.imshow(self.currentScan[minX:maxX, minY:maxY], interpolation="nearest", cmap="Reds",alpha = 0.5)
            self.axGrid.set_title("Grid Map Frame {}".format(self.frame))
        
        plt.pause(0.01)
        
    def gaussKernal(self, kernlen=3, nsig=0.8):
        """
        Returns a 2D Gaussian kernel array.
        Inputs
        ------
        kernlen: length of kernal, must be odd number
        nsig: standard deviation of Gaussian kernal
        
        Note:
        when kerlen = 3, nsig = 0.8, the Gaussian kernal is:
        
        1/16    1/8     1/16
        1/8     1/4     1/8
        1/16    1/8     1/16
        """

        # create nxn zeros
        inp = np.zeros((kernlen, kernlen))
        # set element at the middle to one, a dirac delta
        inp[kernlen//2, kernlen//2] = 1
        # gaussian-smooth the dirac, resulting in a gaussian filter mask
        return fi.gaussian_filter(inp, nsig)
    
    def publishMap(self,kernLen=3,GaussianFilter=False):
        """Final filter the glass map base on grid map and show the result glass map
           Input:
           kernLen: kernal length of gaussian filter
           """
        
        self.finalGlassMap = np.full((4000,4000), 0.5)
        
        # check every gird with occu proba >0, and use Gaussian Mask to register glass info into it
        index = np.where(self.gridMap>0)
        
        if GaussianFilter == True:
            d = kernLen/2
            for i, j in zip(index[0], index[1]):
                self.finalGlassMap[i,j] = np.sum(self.glassMap[i-d:i+d+1, j-d:j+d+1] * self.gaussKernal(kernLen))
            
        if GaussianFilter == False:
            self.finalGlassMap[index] = self.glassMap[index]
        
#        # filter out non-glass grid whose occupancy probability is lower than 25%
#        index2 = np.where((self.gridMap<self.occuThreshold) & (self.finalGlassMap<0.5))
#        for i, j in zip(index2[0], index2[1]):
#            self.finalGlassMap[i,j] = 0.5
        
        # Get the showing range in matplotlib window
        minX = self.mapRange[0] - 50
        maxX = self.mapRange[1] + 50
        minY = self.mapRange[2] - 50
        maxY = self.mapRange[3] + 50
        glassMap = self.finalGlassMap[minX:maxX, minY:maxY]
        
        # =============================================
        # From here: show the glass map
        fig2, ax2 = plt.subplots(1)
        ax2.set_title("Final Glass Confidence Map")
        ax2.cla()
        ax2.imshow(glassMap, interpolation="nearest", cmap="coolwarm")
    
        # =============================================
        # create the new cmap for showing unknown area on the glass map
        colors = [(122.0/255.0, 170.0/255.0, 145.0/255.0, 1.0), (1,1,1, 1.0), (1, 1, 1, 0.0)] # (r, g, b, alpha) [green(-1), white(0), transparent(1)]
        cm = LinearSegmentedColormap.from_list("myCmap", colors, N=len(colors))
        
        # show the occupancy grid map's unknown area in dark green color
        gridMap = np.copy(self.gridMap[minX:maxX, minY:maxY]) # for usage convenience
        maskMap = np.full(gridMap.shape, 1.0)   # mask map on the top of glass map to get desired appearance
        
        maskMap[gridMap==-1] = -1.0               # only keep -1 (unknown), 0 (free) and 1(occupied) in the map
        maskMap[gridMap==0] = 0.0
        
        # Two thresholds filtering: filter out non-glass grid whose occupancy probability is lower than 25%
        index3 = np.where((gridMap<35) & (gridMap>0) ) # & (glassMap<0.7)
        for i, j in zip(index3[0], index3[1]):
            maskMap[i,j] = 0.0 # white
            
        ax2.imshow(maskMap, interpolation="nearest", cmap=cm)

        # Save image to map format with associated *.yaml file
        plt.axis('off')
        plt.tick_params(axis='both', left='off', top='off', right='off', bottom='off', labelleft='off', labeltop='off',
                    labelright='off', labelbottom='off')
        dpi_fig = min(abs(maxX-minX), abs(maxY-minY))
        print('maxX-minX = ', dpi_fig)
        fig2.savefig("Map.png", dpi=dpi_fig, bbox_inches='tight', pad_inches=0)
        yaml_info = {'image' : 'Map.png', 
                     'resolution' : self.mapMetaData.resolution,
                     'origin' : [self.mapMetaData.origin.position.x, self.mapMetaData.origin.position.y, 0.0],
                     'negate' : 0,
                     'occupied_thresh' : 0.65,
                     'free_thresh' : 0.196}

        with open('Map.yaml', 'w') as yaml_file:
            yaml.dump(yaml_info, yaml_file, default_flow_style=False)

        plt.tick_params(axis='both', left='on', top='off', right='off', bottom='on', labelleft='on', labeltop='off',
                    labelright='off', labelbottom='on')
        plt.axis('on')

        # =============================================
        # From here: plot grid map
        fig3, ax3 = plt.subplots(1)
        ax3.set_title("Occupancy Grid Map")
        ax3.cla()
        
        # from grid map build occuGridMap: unknown -1, free 0, occupied 1
        occGridMap = np.copy(self.gridMap[minX:maxX, minY:maxY])
        occThre = 35
        occGridMap[(occGridMap<occThre) & (occGridMap>0)]=0
        occGridMap[occGridMap>=occThre]=1
        
        # build cmap for occGridMap
        colors2 = [(122.0/255.0, 170.0/255.0, 145.0/255.0, 1.0), (1,1,1, 1.0), (0, 0, 0, 1.0)] # (r, g, b, alpha) [green, white, black]
        cm2 = LinearSegmentedColormap.from_list("myCmap2", colors2, N=len(colors2))
        
        ax3.imshow(occGridMap, interpolation="nearest", cmap=cm2)

        # =============================================
        # From here: plot occupancy map that will be used for AMCL
        # fig4, ax4 = plt.subplots(1)
        # ax4.set_title("Occupancy Grid Map AMCL")
        # ax4.cla()
        # colors3 = [(180.0/255.0, 180.0/255.0, 180.0/255.0, 1.0), (1,1,1, 1.0), (80.0/255.0, 80.0/255.0, 80.0/255.0, 1.0)] # (r, g, b, alpha) [gray, white, darkGray, black]

        return
        
    def glassDetectWang (self, senIntensity, triggerIntensity = 4000, intensityDelta = 2000, profileWidth = 5, debug=False):
        """
        Wang's method to detect glass by detecting the peak of the glass
        Input:
        -----
        senIntensity: array (<1080,) LRF's intensity at one certain time point
        
        Output:
        ------
        glassIndex: index of the glass points in this scan, array.shape=(n,)
        """
        glassIndex = []
        for i in range(len(senIntensity)):
            if senIntensity[i] > triggerIntensity:
                if debug==True:
                    print("senIntensity[i]: ", senIntensity[i])
                    print("[i] and [i-1] diff: ", senIntensity[i]-senIntensity[i-1])
                
                if i-1 >= 0 and senIntensity[i]-senIntensity[i-1] >= intensityDelta:
                    j = i
                    while j<len(senIntensity)-1 and senIntensity[j]-senIntensity[j+1] < intensityDelta:
                        
                        if debug ==True:
                             print("diff>threshold, j: ", j, "[j]-[j+1]= ", senIntensity[j]-senIntensity[j+1], 
                                   "\n", "j-i=",j-i)
                        j += 1
                               
                    if j-i < profileWidth:
                        m = (i+j)/2
                        glassIndex.append(m)
        return np.asarray(glassIndex)
        
    def glassMappingWang(self, debug=True):
        """
        Wang's method of building glass map, for comparison with my method
        """
        
        if debug == True: 
            print("========================= Wang's method of glass mapping (Frame = %d)===================="%self.frame)
            
        # get LRF and robot pose information 
        alphaAll = np.arange(self.senMinAng, self.senMaxAng+self.senIncrement/2, self.senIncrement) + np.deg2rad(135)
        senRange = copy.deepcopy(self.senRange) # copy to prevent being changed by LRF callback function in another thread
        senIntensity = copy.deepcopy(self.senIntensity)
        robotPose = copy.deepcopy(self.robotPose)
        
        # delete LRF beans further than maxRange (use or not?)
        alphas = np.delete(alphaAll, np.where(senRange>self.maxRange)) # alpha.shape (<1081,)
        senIntensity = np.delete(senIntensity, np.where(senRange>self.maxRange))
        senRange = np.delete(senRange, np.where(senRange>self.maxRange))
        
        # detect glass using wang's peak detecting method, get glass index
        glassIndex = self.glassDetectWang(senIntensity)
        
        # calculate endpoints' coordinates in the map
        if (len(glassIndex)) == 0: 
            print("Inside glassMappingWang: len(glassIndex)==0, no glass is detected")
            return
        sensorCoord = self.calCoordinate(robotPose, senRange, alphas)
        
        # register the glass to grid map
        glassCoord = sensorCoord[glassIndex]
        mapIndex = np.array(((100-glassCoord[:,1])/0.05, (glassCoord[:,0]+100)/0.05), dtype=int).T
        self.glassMap[self.gridMap==100] = 0 # register the non-glass object
        self.glassMap[mapIndex[:,0],mapIndex[:,1]] = 1.0          # register the glass object
        
        # update the map size
        index = np.array(((100-sensorCoord[:,1])/0.05, (sensorCoord[:,0]+100)/0.05), dtype=int).T
      
        indX = index[:,0]
        indY = index[:,1]
        
        if self.mapRange[0] > np.amin(indX): self.mapRange[0] = np.amin(indX) 
        if self.mapRange[1] < np.amax(indX): self.mapRange[1] = np.amax(indX)
        if self.mapRange[2] > np.amin(indY): self.mapRange[2] = np.amin(indY) 
        if self.mapRange[3] < np.amax(indY): self.mapRange[3] = np.amax(indY) 
        
        if debug == True: 
            print("glassIndex = ", glassIndex, "\nglassCoord: ", glassCoord, "\nmapIndex:", mapIndex)
            

    def glassMappingJiang(self, recordInfo=False):
        """main function for building galss map"""
        
        print("========================= Jiang's method of glass mapping (frame = %d) ===================="%self.frame)
        if(self.senRange is None):
            print("self.senRange is None")
            return
        if(self.senIntensity is None):
            print("self.senIntensity is None")
            return
        if(self.gridMap is None):
            print("self.gridMap is None")
            return
        if(self.glassMap is None):
            print("self.glassMap is None")
            return
        if(self.robotPose is None):
            print("self.robotPose is None")
            return
         
        # remove LRF info whose range longer than maxRange
        alphaAll = np.arange(self.senMinAng, self.senMaxAng+self.senIncrement/2, self.senIncrement) + np.deg2rad(135)
        senRange = copy.deepcopy(self.senRange) # copy to prevent being changed by LRF callback function in another thread
        senIntensity = copy.deepcopy(self.senIntensity)
        robotPose = copy.deepcopy(self.robotPose)
        alphas = np.delete(alphaAll, np.where(senRange>self.maxRange)) # alpha.shape (<1081,)
        senIntensity = np.delete(senIntensity, np.where(senRange>self.maxRange))
        senRange = np.delete(senRange, np.where(senRange>self.maxRange))
        
        # calculate incident angles
        incidAngle = self.calIncidentAngle(senRange, alphas, showH=False, showL=False)
        if incidAngle is None:
            print("After calIncidentAngle: no valid incident angle can be calculated. Finish current loop")
            return
            
        # filter out incorrect incident angle
        len1 = len(incidAngle)
        alphas = alphas[incidAngle>-0.8] # should be == -1, use this considering precision problem
        senRange = senRange[incidAngle>-0.8]
        senIntensity = senIntensity[incidAngle>-0.8]
        incidAngle = incidAngle[incidAngle>-0.8]
        if len(incidAngle) == 0:
            print("After filtering out incorrect angle: no anlge resists, finish current loop")
            return
        print("After filtering incorrect angle: len(incidAngle)=%d, removed=%d"%(len(incidAngle), len1-len(incidAngle)))

        # calculate endpoints' coordinates in the map
        sensorCoord = self.calCoordinate(robotPose, senRange, alphas)
        
        # claculate glass probability
        glassProb = self.classifier(senRange, incidAngle, senIntensity) # shape (<1081,)

        if recordInfo == True:
            # record [0.sensorCoord_X 1.sensorCoord-Y 2.senRange 3.incidAngle 4.senIntensity 5.glassProb] to a big array
            global allGridInfo
            gridInfo = np.concatenate((sensorCoord, senRange.reshape(-1,1), incidAngle.reshape(-1,1), 
                                        senIntensity.reshape(-1,1), glassProb.reshape(-1,1)), axis=1)  # shape (<1081, 6)
            allGridInfo = np.concatenate((allGridInfo, gridInfo), axis = 0)  # shape (<1081*n, 6)
            print("allGridInfo shape: ", allGridInfo.shape)
        
        # sensor endpoints: coordinates -> index in map matrix;  shape (<1081, 2)
        index = np.array(((100-sensorCoord[:,1])/0.05, (sensorCoord[:,0]+100)/0.05), dtype=int).T
      
        # check nearby grids of each beam endpoints
        indX = index[:,0]
        indY = index[:,1]
        
        if self.mapRange[0] > np.amin(indX): self.mapRange[0] = np.amin(indX) 
        if self.mapRange[1] < np.amax(indX): self.mapRange[1] = np.amax(indX)
        if self.mapRange[2] > np.amin(indY): self.mapRange[2] = np.amin(indY) 
        if self.mapRange[3] < np.amax(indY): self.mapRange[3] = np.amax(indY) 
        
        # update current scan for ploting in grid map if want
        self.currentScan.fill(0)
        self.currentScan[indX, indY] = 1        
        
        # Choose method from 1, 2, 3, 4
        method = 3
        
        """
        Method 1 of updating glass map 
        
        - find the endpoint's nearst occupied grid using FLANN 
        - update that grid using the endpoint's glass probability
        - the processing is done per scan (<= 1080 endpoints together)
        - Method 1 is the method used in offline version
        
        Note: method 2 below is considered better than method 1, and method 1 was kept only for testing, not recommend to use
        """
        
#		########## Start of Mehtod 1 ################
        if method == 1:
             # shape (2, n-grids) index of occupied girds
            index1 = np.array(np.where(self.gridMap>=35)) 
            print("shape of index1", index1.shape)
            
             # coordinates of occupied grids in map axis, shape (n-grids,2)
            occupGridCoord = np.array((-100+index1[1]*0.05+0.025, 100-index1[0]*0.05-0.025)).T
            print("shape of occupGridCoord", occupGridCoord.shape)
        
             # find coordinates of nearest occupied grids to each endpoints: gridNearCoord.shape (<1081, 2)
             # for speed reason, put 1081 beams' info together and run FLANN once for each robot_t 
            index2, dists = self.flann.nn(occupGridCoord, sensorCoord, 1, algorithm="kmeans", branching=32, iterations=7, checks=16)
            gridNearCoord = occupGridCoord[index2] # shape (<1081, 2) (x, y)
            print("shape of gridNearCoord", gridNearCoord.shape)
             #statExpress(dists) # print several statistical results

             # update probs in glassMap
            index3 = ((gridNearCoord*[1, -1]+100-0.025)/0.05).astype(int)  # shape(<1081,2)
            indX3 = index3[:,1]
            indY3 = index3[:,0]
 
            self.glassMap[indX3, indY3] = self.updateProb(self.glassMap[indX3, indY3], glassProb)
#         ############ End of method 1 ##############

        """
		Method 2 of updating glass map 

        - check if the endpoint's surrounding 8 grids are occupied, 
        - update occupied grids using the endpoint's glass probability
        - the processing is done per scan (<= 1080 endpoints together)
        """
        ########## Start of Method 2 ################
        if method == 2:
            
            checkIndX = np.array((indX-1, indX-1, indX-1, indX, indX, indX, indX+1, indX+1, indX+1)).T.reshape(-1,)
            checkIndY = np.array((indY-1, indY, indY+1, indY-1, indY, indY+1, indY-1, indY, indY+1)).T.reshape(-1,)
            
            # Check if endpoints' nearby grids are occupied (currently it's not checking actually, or registering all glass info)
            nearOccupyInfo = self.gridMap[checkIndX, checkIndY]
            nearOccupyBool = nearOccupyInfo >= -10  # Unknown = -1; Free = 0; Occupied = 100 (if gmapping publishes probaMap, it from 0 to 100)
            fillIndX = checkIndX[nearOccupyBool]
            fillIndY = checkIndY[nearOccupyBool]

            glassProbRepeat = np.repeat(glassProb, 9) # the order must match the checkIndX/Y order!
            glassProbFill = glassProbRepeat[nearOccupyBool]
            
            self.glassMap[fillIndX, fillIndY] = self.updateProb(self.glassMap[fillIndX, fillIndY], glassProbFill)

        ############ End of method 2 #############
        
        """
        Method 3 of updating glass map
        - keep all glass info, don't filter using grid map's occupancy info, because grid map will shift later anyway
        - use Gaussian Kernal spread a single LRF endpoint's glass prob to nearby block area
        
        - This method, combined with the Gaussian filter usd in PublishMap, considers robot pose uncertainty and LRF measurement uncertainty
        """
        ########## Start of Method 3 ################
        if method == 3:
            
            # find all surrounding grids' index 
            checkIndX = np.array((indX-1, indX-1, indX-1, indX, indX, indX, indX+1, indX+1, indX+1)).T.reshape(-1,)
            checkIndY = np.array((indY-1, indY, indY+1, indY-1, indY, indY+1, indY-1, indY, indY+1)).T.reshape(-1,)
            
            # variously update the probabiblies according a Gaussian Mask weights
            glassProbRepeat = np.repeat(glassProb, 9) # the order must match the checkIndX/Y order!
            kernLen = 3
            b = kernLen/2
            gau = self.gaussKernal(kernLen)
            gauOne = (gau/gau[b,b]).flatten() # Gaussian Mask with the center value is one
            weight = np.repeat(gauOne, len(glassProb))
            glassProbGauss = (glassProbRepeat-0.5)*weight+0.5
            
            self.glassMap[checkIndX, checkIndY] = self.updateProb(self.glassMap[checkIndX, checkIndY], glassProbGauss)

        """
        Method 4 of upadting map
        - only update the beams' endpoints's pointing grid, without any expanding, without using Gaussian Filter 
        - do not consider any robot pose and map uncertainty
        """
        
        ################### Start of Method 4 ###########################
        if method == 4: 
            self.glassMap[indX, indY] = self.updateProb(self.glassMap[indX, indY], glassProb)
        ################### End of Method 4 ##############################

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
    
def statExpress(dists):
    """
    For debuging
    several statistical expression of the FLANN dists 
    """
    print("mean ", np.mean(dists), 
          "median ",np.median(dists),'\n', 
          "ptp ", np.ptp(dists), 
          "amin ", np.amin(dists), 
          "dists ", np.amax(dists),'\n',
          "98% dists ",np.percentile(dists, 98), 
          "var ",np.var(dists), 
          "std ",np.std(dists),'\n',
         "skew ", stats.skew(dists))
    plt.figure()
    plt.hist(dists, bins=40)



def main(recordInfo=True):
    rospy.init_node("glass_map")
    glassMapBuilder = GlassMapBuilder() # initialize frame = 0
    MapPublished = False    # used to make sure publish glass map at the end..
    rate = rospy.Rate(10.0)  # the updating frequency of robotPose and glass map
    
    while not rospy.is_shutdown():
        glassMapBuilder.frame+=1 # must update frame to 1 first then plot glass map
        startTime = time.time()
        glassMapBuilder.getRobotPose()
        glassMapBuilder.glassMappingJiang(recordInfo=recordInfo)
        
        endTime = time.time()
        glassMapBuilder.plotMap()
        
        print("Time for one loop is ", endTime - startTime)
        
        try:
            rate.sleep()
        except:
            print("Publish final glass map after trying rate.sleep() fails.")
            glassMapBuilder.publishMap()
            MapPublished = True
            rospy.signal_shutdown("Shut down Request")
            
    if recordInfo == True:
        global allGridInfo
        np.save("/home/nicolas/catkin_ws/src/tests/scripts/newAllGridInfo.npy", allGridInfo)
        print("End of the main loop. Saved allGridInfo into files")
    
    if MapPublished is False:
        print("Publish final glass map after shutting down correctly")
        glassMapBuilder.publishMap()
        rospy.signal_shutdown("Shut down Request")
        
    plt.show()
    return
    
if __name__=='__main__':
    main()

