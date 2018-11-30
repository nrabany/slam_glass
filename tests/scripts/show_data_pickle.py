"""
This script reads the bag file and calculate the std in function of range and angle
"""
from __future__ import print_function
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from calc_incident_angle import calIncidentAngle
import copy
import sys
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import math
from scipy import stats
import pickle


# Parameters for plots
SCAN_LIMIT = 5000
# ALPHA = 540 - 159 + 51
ALPHA_0 = -10
ALPHA = 540 + int(4*ALPHA_0)
ANGLE_LIMIT = 35

# Replace values that meet a condition by a NaN value
def nan_if(arr, arr_compare, value):
	return np.where(arr_compare > value, np.nan, arr)

# Give $FILE_NAMES withous *.bag* extension as arguments to save the datas as pickletext files with the calculated incident angles
# if len(sys.argv) < 2:
# 	print('Error: At least 1 file name needs to be given as argument.')
# 	quit()

# Axes3D.plot_surface(ranges, angles, sigmas)
# Data for three-dimensional scattered points
# fig = plt.figure()
# ax = plt.axes(projection='3d')
incidentAngles = np.array([])

files = ['height_10second']

for arg in files: # Read $FILE_NAME.txt
	# Data are stored as follow:
	# {
	#   'ranges': (1081xN) array,
	#   'intensities': (1081xN) array,
	#   'angles': (1081xN) array,
	#   'incidentAngles': (1081xN) array,
	# }

	# Read data file
	with open('../datas/data-{}.txt'.format(arg), 'rb') as f:
		scans = pickle.load(f)

	# Use np.nanmean to discard ranges that go through the window
	scans_alpha = scans['ranges'][ALPHA,:]
	# scans_alpha = scans_alpha[scans_alpha<5]

	print('before taking outliers out:',scans_alpha.shape)
	def reject_outliers(data, m = 4):
		d = np.abs(data - np.median(data))
		mdev = np.median(d)
		s = d/mdev if mdev else 0.
		return data[s<m]
		
	# scans_alpha = reject_outliers(scans_alpha)
	print('after taking outliers out:',scans_alpha.shape)
	print(np.std(scans_alpha))

	plt.figure()
	plt.plot(scans_alpha, '.')
	plt.show()

	# Calculate the parameters (sigma) of the gaussian distribution for each scan
	angles = -(np.rad2deg(scans['angles']) - 135*np.ones(1081))
	# Use np.nanmean to discard ranges that go through the window
	range_mean = np.mean(scans['ranges'], axis=1)
	range_mean[range_mean>10] = 10
	nan_ranges = nan_if(scans['ranges'], scans['ranges'], range_mean[:,np.newaxis]+0.5)
	ratio_not_detected = np.sum(np.isnan(nan_ranges), axis=1) / float(np.shape(nan_ranges)[1])
	ranges = np.nanmean(nan_ranges, axis=1)
	sigmas = np.std(nan_ranges, axis=1)
	intensities = np.mean(nan_if(scans['intensities'], scans['ranges'], range_mean[:,np.newaxis]+0.5), axis=1)

	# Filter by angle
	Ratio = ratio_not_detected[angles<ANGLE_LIMIT]
	idata = intensities[angles<ANGLE_LIMIT]
	zdata = sigmas[angles<ANGLE_LIMIT]
	xdata = ranges[angles<ANGLE_LIMIT]
	ydata = angles[angles<ANGLE_LIMIT]

	if arg == 1:
		R = Ratio[ydata>0]
		I = idata[ydata>0]
		Z = zdata[ydata>0]
		X = xdata[ydata>0]
		Y = ydata[ydata>0]
	else: 
		R = np.concatenate((R, Ratio[ydata>0]))
		I = np.concatenate((I, idata[ydata>0]))
		Z = np.concatenate((Z, zdata[ydata>0]))
		X = np.concatenate((X, xdata[ydata>0]))
		Y = np.concatenate((Y, ydata[ydata>0]))

	print('Nb of points in plot: ',np.shape(Z))

# Here change 3rd argument for plotting std, intensity or ratio of non-detection of glass
# ax.scatter3D(X, Y, R, c=R, cmap='rainbow')
# ax.set_xlabel('range (m)')
# ax.set_ylabel('angle (deg)')
# ax.set_zlabel('ratio of non detection')

plt.show()