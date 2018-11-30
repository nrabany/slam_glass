"""
This file reads the data file that was previously saved by read_bag_matrix.py and plot the data
"""
from __future__ import print_function
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy import stats
import sys

# Parameters for plots
SHOW_DATA = True
ANGLE = 'DEG'
ALPHA_1 = 540
ALPHA_2 = 500

# Data are stored as follow:
# {
#   'ranges': (1081xN) array,
#   'intensities': (1081xN) array,
#   'angles': (1081xN) array,
#   'incidentAngles': (1081xN) array,
# }

# Read data file
with open('../datas/data-{}.txt'.format(str(sys.argv[1])), 'rb') as f:
    scans = pickle.load(f)

# Plot data obtained for specific angle
if SHOW_DATA == True:
    f1, ax1 = plt.subplots(1, 3)
    f1.suptitle('Measurements at {} deg'.format(round(np.rad2deg(scans['angles'][ALPHA_1])-135, 2)))
    ax1[0].plot(scans['ranges'][ALPHA_1,:], '.')
    ax1[0].set_title('Ranges')
    ax1[1].plot(scans['intensities'][ALPHA_1,:], '.')
    ax1[1].set_title('Intensities')
    if ANGLE == 'DEG':
        ax1[2].plot(np.rad2deg(scans['incidentAngles'][ALPHA_1,scans['incidentAngles'][ALPHA_1,:]>=0]), '.')
    else:
        ax1[2].plot(scans['incidentAngles'][ALPHA_1,scans['incidentAngles'][ALPHA_1,:]>=0], '.')
    ax1[2].set_title('Incident Angles')

    f2, ax2 = plt.subplots(1, 3)
    f2.suptitle('Measurements at {} deg'.format(round(np.rad2deg(scans['angles'][ALPHA_2])-135, 2)))
    ax2[0].plot(scans['ranges'][ALPHA_2,:], '.')
    ax2[0].set_title('Ranges')
    ax2[1].plot(scans['intensities'][ALPHA_2,:], '.')
    ax2[1].set_title('Intensities')
    if ANGLE == 'DEG':
        ax2[2].plot(np.rad2deg(scans['incidentAngles'][ALPHA_2,scans['incidentAngles'][ALPHA_2,:]>=0]), '.')
    else:
        ax2[2].plot(scans['incidentAngles'][ALPHA_2,scans['incidentAngles'][ALPHA_2,:]>=0], '.')
    ax2[2].set_title('Incident Angles')

# # Calculate the parameters (mu, sigma) of the gaussian distribution for angle = ALPHA_1 
# data = scans['ranges'][ALPHA_1,:]
# mu = np.mean(data)
# sigma = np.std(data)
# x = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
# print('mu =', mu, 'sigma =', sigma)

# # Plot histogram of data and associated gaussian distribution
# plt.figure()
# plt.plot(x,stats.norm.pdf(x, mu, sigma))
# plt.hist(data, density=True)



# Replace values that meet a condition by a NaN value
def nan_if(arr, value):
    return np.where(arr > value, np.nan, arr)

# Calculate the parameters (sigma) of the gaussian distribution for each scan
angles = np.rad2deg(scans['angles']) - 135*np.ones(1081)
# angles = np.rad2deg(np.median(scans['incidentAngles'], axis=1))
# Use np.nanmean to discard ranges that go through the window
ranges = np.nanmean(nan_if(scans['ranges'], 1.4), axis=1)
sigmas = np.std(nan_if(scans['ranges'], 1.4), axis=1)

# Axes3D.plot_surface(ranges, angles, sigmas)
# Data for three-dimensional scattered points
fig = plt.figure()
ax = plt.axes(projection='3d')
# Filter by range
# zdata = sigmas[ranges<1.2]
# xdata = ranges[ranges<1.2]
# ydata = angles[ranges<1.2]
# Filter by angle
zdata = sigmas[np.absolute(angles)<18]
xdata = ranges[np.absolute(angles)<18]
ydata = angles[np.absolute(angles)<18]

print('Nb of points in plot: ',np.shape(xdata))
# print(np.shape(ydata))
# print(np.shape(zdata))


ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='rainbow')
ax.set_xlabel('range (m)')
ax.set_ylabel('angle (deg)')
ax.set_zlabel('sigma (m)')
plt.show()