"""
This script reads the bag file and calculate the std for one laser specified by angle ALPHA
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
from scipy import stats, linalg
from scipy import optimize
from sklearn import linear_model
import ransac 

# Parameters for plots
SCAN_LIMIT = 10000
ALPHA = 500
ANGLE_LIMIT = 40
THRESH_GO_THROUGH = 0.2

# Replace values that meet a condition by a NaN value
def nan_if(arr, arr_compare, value):
	return np.where(arr_compare > value, np.nan, arr)

def mean_if_nan(arr):
	return np.where(np.isnan(arr), np.nanmean(arr), arr)

# Give $FILE_NAMES withous *.bag* extension as arguments to save the datas as pickletext files with the calculated incident angles
# if len(sys.argv) < 2:
# 	print('Error: At least 1 file name needs to be given as argument.')
# 	quit()

elbow_angles = np.array([])
elbow_ranges = np.array([])

files = sys.argv
# files = ['test_8b', 'test_8c', 'test_8d', 'test_8e', 'test_8f', 'test_8g', 'test_8h', 'test_8i', 'test_8j']
files = ['test_8b', 'test_9d']

file_index = 0

for arg in files: # Read $FILE_NAME.bag 
	# bag = rosbag.Bag('../bags/test-g2.bag')
	bag = rosbag.Bag('../bags/test-set3/{}.bag'.format(arg))

	angles = np.linspace(0.0, np.deg2rad(270), num=1081)
	gridSize = 0.05
	maxRange = 29
	n_scans = 0
	# Read the specified topics
	for topic, msg, t in bag.read_messages(topics=['/first', '/scan', '/map_metadata']):

		if topic == '/map_metadata':
			gridSize = round(msg.resolution,2)
		if topic == '/first' or topic == '/scan':
			senRangesAll = np.asarray(copy.deepcopy(msg.ranges))
			senIntensitiesAll = np.asarray(copy.deepcopy(msg.intensities))			

			if n_scans == 0:
				# Calculate incident angles for first scan only (otherwise too long)
				senIndices = np.ones((1081), dtype=bool)
				senIndices[np.where(senRangesAll>maxRange)] = False

				senAngles = angles[senIndices]
				senIntensities = senIntensitiesAll[senIndices]
				senRanges = senRangesAll[senIndices]
				senIncidentAngles = calIncidentAngle(senRanges, senAngles, gridSize, maxRange)

				incidentAngles = np.ones((1081)) * (-1)
				incidentAngles[senIndices] = senIncidentAngles

				# Add new axis to be able to concatenate later
				ranges = senRangesAll[:, np.newaxis]
				intensities = senIntensitiesAll[:, np.newaxis]

			else:
				ranges = np.concatenate((ranges, senRangesAll[:, np.newaxis]), axis=1)
				intensities = np.concatenate((intensities, senIntensitiesAll[:, np.newaxis]), axis=1)
			
			sys.stdout.write("\rconverting... %i" %n_scans)
			sys.stdout.flush()
			# commands to break the loop after i = SCAN_LIMIT
			if n_scans>SCAN_LIMIT:
				break
			n_scans=n_scans+1

	bag.close()
	print('\nData from {}.txt'.format(arg), 'have been extracted.')

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
	# plt.show()

	# Calculate the parameters (sigma) of the gaussian distribution for each scan
	angles_deg = -(np.rad2deg(scans['angles']) - 135*np.ones(1081))
	# Extract values from 0 deg to ANGLE_LIMIT deg
	ranges_glass = scans['ranges'][angles_deg<ANGLE_LIMIT,:]
	intensities_glass = scans['intensities'][angles_deg<ANGLE_LIMIT,:]
	angles_glass = angles_deg[angles_deg<ANGLE_LIMIT]
	ranges_glass = ranges_glass[angles_glass>0,:]
	intensities_glass = intensities_glass[angles_glass>0,:]
	angles_glass = angles_glass[angles_glass>0]

	# Use np.nanmean to discard ranges that go through the window
	range_mean = np.mean(ranges_glass, axis=1)
	range_mean[range_mean>6] = 6
	nan_ranges = nan_if(ranges_glass, ranges_glass, range_mean[:,np.newaxis]+0.5)
	# Calculate ratio of scans that goes through the window
	ratio_not_detected_filt = np.sum(np.isnan(nan_ranges), axis=1) / float(np.shape(nan_ranges)[1])

	# Extract angle where scans begin to go through the window
	elbow_angle = np.amin(angles_glass[ratio_not_detected_filt>THRESH_GO_THROUGH])
	elbow_angles = np.append(elbow_angles, elbow_angle)

	# Extract values that are below elbow, filter a second time
	nan_ranges = nan_if(nan_ranges, angles_glass[:,np.newaxis], elbow_angle)

	def reject_outliers(data, m = 4):
		d = np.abs(data - np.nanmedian(data))
		mdev = np.nanmedian(d)
		s = d/mdev if mdev else 0.
		# return data[s<m]
		return np.where(s>m, np.nan, data)
	
	nan_ranges = reject_outliers(nan_ranges)

	# Calculate average range, std of range and average intensity during the time
	ranges_filt = np.nanmean(nan_ranges, axis=1)
	ranges_filt = mean_if_nan(ranges_filt)
	sigmas_filt = np.nanstd(nan_ranges, axis=1)
	intensities_filt = np.nanmean(nan_if(intensities_glass, angles_glass[:,np.newaxis], elbow_angle), axis=1)
	min_filt = np.amin(nan_ranges, axis=1)
	max_filt = np.amax(nan_ranges, axis=1)


	# Calculate average range 
	elbow_range = np.nanmean(ranges_filt)
	elbow_ranges = np.append(elbow_ranges, elbow_range)

	# Add values to array that will be plot at the end
	if file_index == 0:
		R = ratio_not_detected_filt
		I = intensities_filt
		Z = sigmas_filt
		X = ranges_filt
		Y = angles_glass
		print(X.shape)
		# plt.figure()
		# plt.plot(Y,min_filt,'.')
		# plt.plot(Y,max_filt,'.')
		# plt.figure()
		# plt.plot(Y,sigmas_filt,'.')
		# plt.show()
	else: 
		R = np.concatenate((R, ratio_not_detected_filt))
		I = np.concatenate((I, intensities_filt))
		Z = np.concatenate((Z, sigmas_filt))
		X = np.concatenate((X, ranges_filt))
		Y = np.concatenate((Y, angles_glass))

	print('Nb of points in plot: ',np.shape(Z))
	# print(list(Z))
	# print(np.argwhere(Z<0.01036))

	file_index += 1

# Here change 3rd argument for plotting std, intensity or ratio of non-detection of glass
# Axes3D.plot_surface(ranges, angles, sigmas)
# Plot of the std (in function of the range and angle)
fig1 = plt.figure()
ax1 = plt.axes(projection='3d')
ax1.scatter3D(X, Y, Z, c=Z, cmap='rainbow')
ax1.set_xlabel('range (m)')
ax1.set_ylabel('angle (deg)')
ax1.set_zlabel('std (m)')

# Plot of intensity (in function of range and angle)
fig2 = plt.figure()
ax2 = plt.axes(projection='3d')
ax2.scatter3D(X, Y, I, c=I, cmap='rainbow')
ax2.set_xlabel('range (m)')
ax2.set_ylabel('angle (deg)')
ax2.set_zlabel('intensity')

# Plot of ratio glass / non-glass (in function of range and angle)
fig3 = plt.figure()
ax3 = plt.axes(projection='3d')
ax3.scatter3D(X, Y, R, c=R, cmap='rainbow')
ax3.set_xlabel('range (m)')
ax3.set_ylabel('angle (deg)')
ax3.set_zlabel('ratio')

# Plot of std (in function of range and intensity)
fig5 = plt.figure()
ax5 = plt.axes(projection='3d')
ax5.scatter3D(X, I, Z, c=Z, cmap='rainbow')
ax5.set_xlabel('range (m)')
ax5.set_ylabel('intensity')
ax5.set_zlabel('std (m)')

# -----------Plot of threshold of glass detection angle in function of range-----------#
# Compute linear regression for function approximation of glass detection in function of range and distance
"""
X2 = elbow_ranges[:,np.newaxis]
A2 = np.concatenate((X2*X2*X2*X2, X2*X2*X2, X2*X2, X2, np.ones(np.shape(X2))), axis=1)
b2 = elbow_angles

lu2, piv2 = linalg.lu_factor(np.dot(np.transpose(A2), A2))
factors2 = linalg.lu_solve((lu2, piv2), np.dot(np.transpose(A2), b2))

xxx = np.linspace(np.amin(elbow_ranges), np.amax(elbow_ranges))
yyy = xxx*xxx*xxx*xxx*factors2[0] + xxx*xxx*xxx*factors2[1] + xxx*xxx*factors2[2] + xxx*factors2[3] + factors2[4]

def exponential_func(x, a, b, c):
    return a*np.exp(-b*x)+c

popt, pcov = optimize.curve_fit(exponential_func, elbow_ranges, elbow_angles, p0=(1, 1e-6, 1))

yyy = exponential_func(xxx, *popt)
# print('exponential coef: ', *popt)
fig4 = plt.figure()
plt.plot(elbow_ranges, elbow_angles, 'o', xxx, yyy)
plt.xlabel('distance (m)')
plt.ylabel('angle (deg)')
plt.title('glass detection threshold')
"""
# -------------------------------------END---------------------------------------------#

# -----Plot of approximation of std in function of range and distance ------#
# Eliminate NaN values
I = I[np.invert(np.isnan(Z))]
X = X[np.invert(np.isnan(Z))]
Y = Y[np.invert(np.isnan(Z))]
Z = Z[np.invert(np.isnan(Z))]

# Compute linear regression for hyperplane approximation of std
# For range > RANGE_THRESHOLD
RANGE_THRESHOLD = 2 
# RANGE_THRESHOLD = np.amax(X)
Z0 = Z[X>RANGE_THRESHOLD]
Y0 = Y[X>RANGE_THRESHOLD]
X0 = X[X>RANGE_THRESHOLD]
mse0 = 0
mse1 = 0
if X0.shape[0]>0:
	# Create a meshgrid but according to detectable angle and range
	xx, yy = np.meshgrid(np.linspace(RANGE_THRESHOLD, np.amax(X0), 10), np.linspace(np.min(Y0), np.max(Y0), 10))
	indices = np.asarray(np.where(yy>exponential_func(xx[1,:], *popt)+3))
	for ix in range(indices.shape[1]):
		yy[indices[:,ix][0],indices[:,ix][1]] = exponential_func(xx[indices[:,ix][0],indices[:,ix][1]], *popt)

	A = np.concatenate((X0[:,np.newaxis], Y0[:,np.newaxis], np.ones(np.shape(X0))[:,np.newaxis]), axis=1)
	b = Z0

	"""
	# Linear regression
	lu, piv = linalg.lu_factor(np.dot(np.transpose(A), A))
	factors = linalg.lu_solve((lu, piv), np.dot(np.transpose(A), b))

	zz = factors[0]*xx + factors[1]*yy + factors[2]
	ax1.plot_wireframe(xx, yy, zz, cmap='rainbow')
	"""

	# RANSAC
	x = A
	y = Z0.reshape(-1,1)
	all_data = np.concatenate((x, Z0[:,np.newaxis]), axis=1)
	debug = False
	n_input = x.shape[1]
	n_output = y.shape[1]
	model = ransac.LinearLeastSquaresModel(n_input,n_output, debug=debug)
	ransac_fit, ransac_data = ransac.ransac(all_data,model,
									10, 1000, 1, 30, # misc. parameters
									debug=debug,return_all=True)
	print('ransac_fit: ', ransac_fit)

	def hyperplane(x, y, coef):
		a, b, c = coef
		return a*x + b*y + c

	zz = hyperplane(xx, yy, ransac_fit)
	mse0 = np.sum(np.square(Z0-hyperplane(X0,Y0,ransac_fit)))
	print('MSE0 =',mse0)
	ax1.plot_wireframe(xx, yy, zz, cmap='rainbow')


# Compute approximation surface of std for range < RANGE_THRESHOLD
offset = 0.3
Z1 = Z[X<=RANGE_THRESHOLD+offset]
Y1 = Y[X<=RANGE_THRESHOLD+offset]
X1 = X[X<=RANGE_THRESHOLD+offset]

if X1.shape[0]>0:
	# Z1 = Z
	# Y1 = Y
	# X1 = X
	# RANGE_THRESHOLD = np.amax(X)

	# Z1 = 20
	# X1 = 20
	# Y1 = 20

	def quadratic(x, y, coef):
		a, b, c, d, e, f = coef
		return a*x*x + b*y*y + c*x*y + d*x + e*y + f

	def sig2(x, y):
		theta = -1.4
		a = 1
		return 0.014 * (1- 1 / (1 + np.exp(a*((np.cos(theta)*(y-13)-np.sin(theta)*x)))))  + 0.004

	def sigmoid2(x, y, coef):
		a,b,c,d,e,theta = coef
		return a * (1 - 1 / (1 + np.exp(b*((np.cos(theta)*(y-c)-np.sin(theta)*(x-d))))))  + e

	def sigmoid(x, y, coef):
		a,b,c,d,e,f = coef
		return a * (1 - 1 / (1 + np.exp((b*(y-c)+d*(x-e)))))  + f


	def func2min(coef):
		# a, b, c, d, e, f, g, h = coef
		return np.sum(np.square(Z1 - sigmoid(X1, Y1, coef)))

	# ini_guess = [0.014, 1, 13, 0, 0.004, -1.4]
	ini_guess = [0.014, 0.17, 13, 0.98, 0, 0.004]

	coef = optimize.minimize(func2min, ini_guess)
	print('coef sigmoid:', coef.x)

	xx1, yy1 = np.meshgrid(np.linspace(np.amin(X1), RANGE_THRESHOLD, 20), np.linspace(np.min(Y1), np.max(Y1), 20))

	offset = 0
	# zz1 = sigmoid(xx1, yy1, coef.x) + offset
	zz1 = sigmoid(xx1, yy1, coef.x) + offset

	mse1 = np.sum(np.square(Z1-(sigmoid(X1,Y1,coef.x)+offset)))
	print('MSE1 =',mse1)

	ax1.plot_wireframe(xx1, yy1, zz1, cmap='rainbow')


print('MSE total =',mse0+mse1)

# -------------------------------------END---------------------------------------------#


plt.close(fig2)
plt.close(fig3)
plt.close(fig5)
# plt.close(fig4)
plt.show()