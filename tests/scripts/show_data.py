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

# Parameters for plots
SCAN_LIMIT = 5000
# ALPHA = 540 - 159 + 51
ALPHA_0 = -30
ALPHA = 540 + int(4*ALPHA_0)
ANGLE_LIMIT = 35

# Replace values that meet a condition by a NaN value
def nan_if(arr, arr_compare, value):
	return np.where(arr_compare > value, np.nan, arr)

# Give $FILE_NAMES withous *.bag* extension as arguments to save the datas as pickletext files with the calculated incident angles
if len(sys.argv) < 2:
	print('Error: At least 1 file name needs to be given as argument.')
	quit()

# Axes3D.plot_surface(ranges, angles, sigmas)
# Data for three-dimensional scattered points
# fig = plt.figure()
# ax = plt.axes(projection='3d')
incidentAngles = np.array([])
for arg in range(1, len(sys.argv)):# Read $FILE_NAME.bag 
	# bag = rosbag.Bag('../bags/test-g2.bag')
	bag = rosbag.Bag('../bags/test-set3/{}.bag'.format(str(sys.argv[arg])))

	angles = np.linspace(0.0, np.deg2rad(270), num=1081)
	gridSize = 0.05
	maxRange = 29
	i = 0
	first = True
	# Read the specified topics
	for topic, msg, t in bag.read_messages(topics=['/scan', '/first']):
		# if topic == '/map_metadata':
		# 	gridSize = round(msg.resolution,2)
		if topic == '/scan' or topic == '/first':
			senRangesAll = np.asarray(copy.deepcopy(msg.ranges))
			senIntensitiesAll = np.asarray(copy.deepcopy(msg.intensities))			

			if i == -1:
				# Calculate incident angles for first scan only (otherwise too long)
				senIndices = np.ones((1081), dtype=bool)
				senIndices[np.where(senRangesAll>maxRange)] = False

				senAngles = angles[senIndices]
				senIntensities = senIntensitiesAll[senIndices]
				senRanges = senRangesAll[senIndices]
				senIncidentAngles = calIncidentAngle(senRanges, senAngles, gridSize, maxRange)

				incidentAngles = np.ones((1081)) * (-1)
				incidentAngles[senIndices] = senIncidentAngles

			if first == True:
				# Add new axis to be able to concatenate later
				ranges = senRangesAll[:, np.newaxis]
				intensities = senIntensitiesAll[:, np.newaxis]
				first = False
			else:
				ranges = np.concatenate((ranges, senRangesAll[:, np.newaxis]), axis=1)
				intensities = np.concatenate((intensities, senIntensitiesAll[:, np.newaxis]), axis=1)
			
			sys.stdout.write("\rconverting... %i" %i)
			sys.stdout.flush()
			# commands to break the loop after i = SCAN_LIMIT
			if i>SCAN_LIMIT:
				break
			i=i+1

	bag.close()
	print('\nData from {}.txt'.format(str(sys.argv[arg])), 'have been extracted.')

	scans = {'ranges': ranges, 'intensities': intensities, 'angles': angles, 'incidentAngles': incidentAngles}

	# # Calculate the parameters (mu, sigma) of the gaussian distribution 
	# data0 = scans['ranges'][ALPHA,:]
	# mu0 = np.mean(data0)
	# sigma0 = np.std(data0)
	# x0 = np.linspace(mu0 - 3*sigma0, mu0 + 3*sigma0, 100)
	# print('mu =', mu0, 'sigma =', sigma0)
	# print('incident angle =', np.rad2deg(incidentAngles[ALPHA]))
	# # Plot histogram of data and associated gaussian distribution
	# plt.figure()
	# plt.plot(data0, '.')
	# plt.plot(x0,stats.norm.pdf(x0, mu0, sigma0))
	# plt.hist(data0, density=True)

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