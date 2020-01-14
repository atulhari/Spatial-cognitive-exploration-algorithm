#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from shapely.geometry import Polygon, Point
from polygon_Mapper import polygonMapper
from shapely import speedups
from scipy.spatial import KDTree, distance
import random
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # or any {'0', '1', '2'}
import tensorflow as tf
import rospy
import gpflow
from skimage.feature import peak_local_max

speedups.enable()
class gprSpatial(object):
	"""docstring for gprSpatial"""
	def __init__(self):
		rospy.loginfo("SCEAM algorithm initiated ")


	def calcVisibility(self,X):
		global global_polygon
		x = np.atleast_2d(X)
		rospy.loginfo("Num training samples: %s",len(x)) 
		depth = [Point(x[i]).distance(global_polygon.exterior) if global_polygon.contains(Point(x[i])) else 0 for i in range (0,len(x))]
		dstacked = np.array(depth)
		normalized = (dstacked-min(dstacked))/(max(dstacked)-min(dstacked))
		return normalized[:,None]


	def gpRegression(self,training_samples_,global_polygon_):
		global global_polygon
		training_samples = training_samples_
		global_polygon = global_polygon_
		self.status = False
		test_mesh_size = 100
		min_depth = 6
	#----------------------Training -----------------------------------------------#
		X2 =  np.array(training_samples)
		Y2 = self.calcVisibility(X2)
		X2 = np.atleast_2d(X2)
	#------------------------------------------------------------------------------#

	#-------------------------Construct visibility_GP_model_----------------------------------#
		kernel_matern = gpflow.kernels.Matern52(2,active_dims=[0,1], lengthscales=2, ARD=True)
		visibility_GP= gpflow.gpr.GPR(X2, Y2, kern=kernel_matern)
		visibility_GP.likelihood.variance = 0.01
		visibility_GP.compile()  
		rospy.loginfo("visibility GP constructed")
	#------------------------------------------------------------------------------------#

	#--------------------------Testing------------------------------------------------------#
		# test_mesh_size = 100
		minx, miny, maxx, maxy = global_polygon.bounds
		x1 = np.linspace(minx, maxx, test_mesh_size)
		x2 = np.linspace(miny, maxy, test_mesh_size)
		self.x1_mesh,self.x2_mesh = np.meshgrid(x1, x2)
		visibility_mesh =  np.dstack([self.x1_mesh, self.x2_mesh]).reshape(-1, 2)
		posterior_mean_VM, posterior_var_VM = visibility_GP.predict_y(visibility_mesh)
	#---------------------------------------------------------------------------------------#
		mean_square_VM = np.reshape(posterior_mean_VM,self.x1_mesh.shape) # Shape: (test_mesh_size,test_mesh_size)
		coordinates = peak_local_max(mean_square_VM, min_distance=min_depth)

		# UNCOMMENT TO PLOT
		# plt.plot(x1[coordinates[:, 1]], x2[coordinates[:, 0]], 'ko')
		# plt.plot(*global_polygon.exterior.xy, c ='w')
		# im1=plt.contourf(self.x1_mesh, self.x2_mesh, mean_square_VM, cmap = 'jet', alpha= 0.7)
		# plt.colorbar(im1)
		# plt.show()

		self.status = True
		self.x_goal = zip(list(x1[coordinates[:, 1]]), list(x2[coordinates[:, 0]]))
		return self.x_goal

	#-----------------

