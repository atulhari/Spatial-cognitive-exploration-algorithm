#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from shapely.geometry import Polygon, Point
from shapely import speedups
from polygon_Mapper import polygonMapper
from scipy.spatial import KDTree, distance
import random
import numpy as np
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
		rospy.loginfo("iSCEAM algorithm initiated")

	def nearestPointToQuery(self,data,query_point,r_):
		# print data
		T = KDTree(data)
		idx = T.query_ball_point(query_point,r=r_)
		return len(idx)
		# return [data[i] for i in idx][1:-1]


	def occupancyProbabilityCalculator(self,X):
		global entropy
		x = np.atleast_2d(X)
		rospy.sleep(1)
		a2  = [entropy[i]*0.01 for i in range(0,len(x))] 
		out = np.array(a2)
		rospy.loginfo("Num informative training samples: %s",len(out))
		return out[:,None].astype(np.float64)
		# normalized = (out-min(out))/(max(out)-min(out))
		# return normalized[:,None]


	def calcVisibility(self,X):
		global global_polygon
		x = np.atleast_2d(X)
		rospy.loginfo("Num visibility training samples: %s",len(x))
		depth = [Point(x[i]).distance(global_polygon.exterior) if global_polygon.contains(Point(x[i])) else 0 for i in range (0,len(x))]
		dstacked = np.array(depth)
		normalized = (dstacked-min(dstacked))/(max(dstacked)-min(dstacked))
		return normalized[:,None]


	def gpRegression(self,vis_training_samples,entropy_,training_samples_,global_polygon_):
		global entropy,global_polygon
		training_samples = training_samples_
		entropy = entropy_
		global_polygon = global_polygon_
		self.status = False



	#--------------------- Occupancy information training --------------------------------------------#
		
		X1 = np.array(vis_training_samples)
		Y1 = self.occupancyProbabilityCalculator(X1)	
		X1 = np.atleast_2d(X1)		
		sigma_n = 0.01
		print ("occupancyProbabilityCalculator:",Y1.shape)

	#-----------------------------------------------------------------------------#

	#----------------------Visibility training-----------------------------------------------#

		X2 =  np.array(training_samples)
		Y2 = self.calcVisibility(X2)
		X2 = np.atleast_2d(X2)
		print ("calcVisibility:",Y2.shape)
	#------------------------------------------------------------------------------#
	#-----------------------Construct visibility GP-----------------------------#
		k2 = gpflow.kernels.Matern52(2,active_dims=[0,1], lengthscales=2, ARD=True)
		visibility_GP= gpflow.gpr.GPR(X2, Y2, kern=k2)
		visibility_GP.likelihood.variance = 0.01
		visibility_GP.compile()
		rospy.loginfo("visibility GP constructed")
	#-----------------------------------------------------------------------------------------#


	#-------------------------Construct informative GP------------------------------#

		k1 = gpflow.kernels.Matern52(2,active_dims=[0,1], lengthscales=3, ARD=True)
		informative_GP= gpflow.gpr.GPR(X1, Y1, kern=k1)
		informative_GP.likelihood.variance = 0.01
		informative_GP.compile()  
		rospy.loginfo("Informative GP constructed")
	#------------------------------------------------------------------------------------#
	
	#--------------------------Testing---------------------------------------------------# 	
		test_mesh_size = 150
		minx, miny, maxx, maxy = global_polygon.bounds
		x1 = np.linspace(minx, maxx, test_mesh_size)
		x2 = np.linspace(miny, maxy, test_mesh_size)
		self.x1_mesh,self.x2_mesh = np.meshgrid(x1, x2)
		grid =  np.dstack([self.x1_mesh, self.x2_mesh]).reshape(-1, 2)
		self.postirior_mean_VM, var2 = visibility_GP.predict_y(grid)
		self.postirior_mean_IM, var1 = informative_GP.predict_y(grid)
	#-------------------------------------------------------------------------------------#
		mean_square_VM = np.reshape(self.postirior_mean_VM,self.x1_mesh.shape)
		mean_square_IM = np.reshape(self.postirior_mean_IM,self.x1_mesh.shape)
		f_kernel =  np.add(mean_square_VM,mean_square_IM)

		coordinates = peak_local_max(mean_square_IM, min_distance=3)
		self.x_goal_k1 = zip(list(x1[coordinates[:, 1]]), list(x2[coordinates[:, 0]]))

		coordinates2 = peak_local_max(f_kernel, min_distance=6)
		self.x_goal_k2 = zip(list(x1[coordinates2[:, 1]]), list(x2[coordinates2[:, 0]]))
		print("x_goal_k1 :",self.x_goal_k1)
	
		if len(self.x_goal_k1) ==0:
			print "no goals found"
			self.x_goal = self.x_goal_k2
		else:
			print(" Goal from kernel",self.x_goal_k2)
			self.x_goal =[self.x_goal_k2[i] for i in range(0,len(self.x_goal_k2)) if self.nearestPointToQuery(self.x_goal_k1,self.x_goal_k2[i],3)>0]
		 	if len(self.x_goal) ==0:
		 		print "visibility based goal"
				self.x_goal = self.x_goal_k2

		self.status = True
		return self.x_goal

	#-----------------

