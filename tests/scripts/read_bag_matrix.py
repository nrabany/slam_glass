"""
This script reads the bag files and save it to a data file with the incident angles included
"""
from __future__ import print_function
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from calc_incident_angle import calIncidentAngle
import copy
import pickle
import sys

# Parameters
SCAN_LIMIT = 5

# Give $FILE_NAMES withous *.bag* extension as arguments to save the datas as pickletext files with the calculated incident angles
# if len(sys.argv) < 2:
# 	print('Error: At least 1 file name needs to be given as argument.')
# 	quit()

# files = ['test_9a', 'test_9b', 'test_9c', 'test_9d', 'test_9e', 'test_9f', 'test_9g', 'test_9h', 'test_9i', 'test_9j', 'test_9k', 'test_9l', 'test_9m']
# files = ['test_8b', 'test_8c', 'test_8d', 'test_8e', 'test_8f', 'test_8g', 'test_8h', 'test_8i', 'test_8j']
# files = ['test_11a', 'test_11b', 'test_11c', 'test_11d', 'test_11e', 'test_11f', 'test_11g', 'test_11h', 'test_11i', 'test_11j']
# files = ['hight_0', 'hight_1', 'hight_2', 'hight_3', 'hight_4', 'hight_5', 'hight_6', 'hight_7', 'hight_8', 'hight_9']
# files = ['height_3', 'height_4', 'height_5', 'height_6', 'height_7', 'height_8', 'height_9', 'height_10', 'height_11']
# files = ['height_8second']
# files = ['height_3second', 'height_4second', 'height_5second', 'height_6second', 'height_7second', 'height_9second', 'height_10second', 'height_11second']
# files = ['height_clean']
files = ['window4', 'window5']

for arg in files: # Read $FILE_NAME.bag 
	# bag = rosbag.Bag('../bags/test-g2.bag')
	bag = rosbag.Bag('../bags/test-set3/{}.bag'.format(arg))

	# Log out all the topics
	# topic_list = list()
	# for topic, msg, t in bag.read_messages():
	# 	if topic not in topic_list:
	# 		topic_list.append(topic)
	# 		print(topic)


	angles = np.arange(0.0, np.deg2rad(270), np.deg2rad(270)/1081.0)
	gridSize = 0.05
	maxRange = 29
	i = 0
	# Read the specified topics
	for topic, msg, t in bag.read_messages(topics=['/first', '/scan']):
		if topic == '/first' or topic == '/scan':
			if i == 0:
				#here print message to see if topic exists
				# print(msg)
				pass

			senRangesAll = np.asarray(copy.deepcopy(msg.ranges))
			senIntensitiesAll = np.asarray(copy.deepcopy(msg.intensities))

			# senIndices = np.ones((1081), dtype=bool)
			# senIndices[np.where(senRangesAll>maxRange)] = False

			# senAngles = angles[senIndices]
			# senIntensities = senIntensitiesAll[senIndices]
			# senRanges = senRangesAll[senIndices]
			# senIncidentAngles = calIncidentAngle(senRanges, senAngles, gridSize, maxRange)

			# senIncidentAnglesAll = np.ones((1081)) * (-1)
			# senIncidentAnglesAll[senIndices] = senIncidentAngles

			if i == 0:
				ranges = senRangesAll[:, np.newaxis]
				intensities = senIntensitiesAll[:, np.newaxis]
				# incidentAngles = senIncidentAnglesAll[:, np.newaxis]
			else:
				ranges = np.concatenate((ranges, senRangesAll[:, np.newaxis]), axis=1)
				intensities = np.concatenate((intensities, senIntensitiesAll[:, np.newaxis]), axis=1)
				# incidentAngles = np.concatenate((incidentAngles, senIncidentAnglesAll[:, np.newaxis]), axis=1)
			
			sys.stdout.write("\rconverting... %i" %i)
			sys.stdout.flush()
			i=i+1
		# commands to break the loop after i = SCAN_LIMIT
		# if i>SCAN_LIMIT:
		# 	break

	bag.close()

	# Save extracted data for skipping the long incident angles calculations
	# Data are stored as follow:
	# {
	#   'ranges': (1081xN) array,
	#   'intensities': (1081xN) array,
	#   'angles': (1081xN) array,
	#   'incidentAngles': (1081xN) array,
	# }
	scans = {'ranges': ranges, 'intensities': intensities, 'angles': angles}
	with open('../datas/data-{}.txt'.format(arg), 'wb') as f:
		pickle.dump(scans, f)

	print('\ndata-{}.txt'.format(arg), 'has been saved.')




