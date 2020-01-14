#!/usr/bin/env python

import sys
from geometry_msgs.msg import PoseStamped, Point, PolygonStamped, Point32
from nav_msgs.msg import OccupancyGrid,Path
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import String
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
import numpy as np
from shapely.geometry import Polygon, Point
from scipy.spatial import KDTree, distance
from simplification.cutil import simplify_coords
import random
from aco_tsp import SolveTSPUsingACO
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # or any {'0', '1', '2'}

from gpr_spatial_gph import gprSpatial



c_samples = [0.40,0.66,0.19]
s_samples = [0.4,0.4,0.4]
c_small_samples = [0.9,0.67,0.0]
s_small_samples = [0.3,0.3,0.3]


class explore(object):
	def __init__(self):

		print " Node initiated "
		self.status_handle = rospy.Subscriber("map_status", String, self.statusCallback)
		self.global_f_sub = rospy.Subscriber("trajectory_node",MarkerArray, self.trajectoryCallback)
		self.path_sub = rospy.Subscriber("trajectory",Path, self.pathCallback)
		self.poly_global_sub = rospy.Subscriber("polygon_map", PolygonStamped, self.globalPolygonCallback)
		self.poly_local_pub = rospy.Publisher("local_polygon_map", PolygonStamped, queue_size=50)
		self.hs_candidate_pub = rospy.Publisher('hotspot_candidate', MarkerArray, queue_size=10)


	def statusCallback(self,status_handle): # MAP AVAILABILITY
		
		self.map_status = status_handle.data
		map_status = self.map_status

	def trajectoryCallback(self,global_f_sub):	#probabilistic points 
		
		globalfx = [m.pose.position.x for m in global_f_sub.markers] 
		globalfy = [m.pose.position.y for m in global_f_sub.markers]
		self.value = [m.pose.position.z for m in global_f_sub.markers]
		self.global_trajectory = zip(globalfx,globalfy)

	def pathCallback(self,path_sub):  #POSITION | TRAJECTORY 
		
		self.current_position = (path_sub.poses[-1].pose.position.x,path_sub.poses[-1].pose.position.y)

	def globalPolygonCallback(self,poly_global_sub):
		
		polyGx= [m.x for m in poly_global_sub.polygon.points] 
		polyGy = [m.y for m in poly_global_sub.polygon.points]
		self.global_map = zip(polyGx,polyGy)
		self.global_map_s = simplify_coords(np.vstack(self.global_map),0.2)
		self.global_polygon = Polygon(self.global_map_s)

	def probMapCallback(self):

		mapData = rospy.wait_for_message("/map",OccupancyGrid)
		data = mapData.data
		# rospy.loginfo("Got map!")
		w=mapData.info.width
		h=mapData.info.height
		resolution=mapData.info.resolution
		Xstartx=mapData.info.origin.position.x
		Xstarty=mapData.info.origin.position.y
		self.entropy = [data[i*w+j] for i in range(0,h) for j in range(0,w) if data[i*w+j]>10 and data[i*w+j]<50]
		self.info_training_samples = [((j*resolution+ Xstartx),(i*resolution +Xstarty)) for i in range(0,h) for j in range(0,w) if data[i*w+j]>10 and data[i*w+j]<50]		
		self.training_samples = [((j*resolution+ Xstartx),(i*resolution +Xstarty)) for i in range(0,h) for j in range(0,w) if data[i*w+j]>-1 and data[i*w+j]<50]


	def publishMarker(self,msg,c,s,size):
		
		count = 0
		MARKERS_MAX = 100
		markerArray = MarkerArray()
		# training_samples = poly2
		if size > 1:
			for p in (msg):
				
				markers = Marker()
				markers.header.frame_id = '/map'
				markers.header.stamp = rospy.Time.now()
				markers.ns = "frontier_segment"
				markers.action = markers.ADD
				markers.type = markers.SPHERE
				# markers.id = 0

				markers.scale.x = s[0]
				markers.scale.y = s[1]
				markers.scale.z = s[2]
				#markers.scale.y = 0.1

				markers.color.a = 1.0
				markers.color.r = c[0]
				markers.color.g = c[1]
				markers.color.b = c[2]

				markers.pose.position.x = p[0]
				markers.pose.position.y = p[1]
				markers.pose.position.z = 0.5
				markerArray.markers.append(markers)

		elif size ==1:
			markers = Marker()
			markers.header.frame_id = '/map'
			markers.header.stamp = rospy.Time.now()
			markers.ns = "hotspot_candidate"
			markers.action = markers.ADD
			markers.type = markers.SPHERE
			# markers.id = 0

			markers.scale.x = s[0]
			markers.scale.y = s[1]
			markers.scale.z = s[2]
			#markers.scale.y = 0.1

			markers.color.a = 1.0
			markers.color.r = c[0]
			markers.color.g = c[1]
			markers.color.b = c[2]

			markers.pose.position.x = msg[1,0]
			markers.pose.position.y = msg[1,1]
			markers.pose.position.z = 0.5
			markerArray.markers.append(markers)

		if(count > MARKERS_MAX):
			markerArray.markers.pop(0)


		id = 0

		for m in markerArray.markers:
			m.id = id
			id += 1
		return markerArray

	def sampler(self,a):

		ts = []
		sam_num = 300
		s = np.arange(len(a))
		rs = random.sample(s,sam_num)
		ts = [a[rs[i]] for i in range(len(rs))]
		return ts

	def startMovebase(self):

		try:
			result = self.movebase_client()
			if result:
				rospy.loginfo("Goal execution done!")
		except rospy.ROSInterruptException:
			rospy.loginfo("Navigation test finished.")

	def nearestPointToQuery(self,data,query_point):

		T = KDTree(data)
		dist,idx= T.query(query_point,1)
		if dist > self.search_dist:
			return True
		else: 
			return False


	def explorationPlanner(self):

		gp = gprSpatial()
		rospy.sleep(2)
		# goal_store = [self.current_position]
		self.search_dist = 3
		self.goal_list = [self.current_position]
		while not rospy.is_shutdown():
			# rospy.sleep(3)
			gp = gprSpatial()
			self.probMapCallback()
			min_search_dist = 1

			if self.map_status == "done":
				t_sample = self.sampler(self.training_samples)
				exploration_goals = gprSpatial.gpRegression(gp,self.info_training_samples,self.entropy,t_sample,self.global_polygon)
				global_poly = self.global_polygon.buffer(0.2)
				goals = [(exploration_goals[i][0],exploration_goals[i][1]) for i in range(0,len(exploration_goals)) if global_poly.contains(Point((exploration_goals[i][0],exploration_goals[i][1]))) and self.nearestPointToQuery(self.goal_list,(exploration_goals[i][0],exploration_goals[i][1]))]
				hs_candidate_markers = self.publishMarker(goals,c_small_samples,s_small_samples,size = 2)
				self.hs_candidate_pub.publish(hs_candidate_markers)
				#decision making TSP-ACO
				if len(goals)>1:
					# Setup parameters
					_colony_size = 10
					_steps = 10

					# Select mode
					# ['ACS', 'Elitist', 'MaxMin']
					_mode = 'ACS'
					_nodes = goals
					# Model setup and run
					model = SolveTSPUsingACO(
					    mode = _mode,
					    colony_size = _colony_size,
					    steps = _steps,
					    nodes = _nodes
					)
					runtime, distancr_2 = model.run()
					route = model.global_best_tour
					print "Global best tour is:",route

					self.goal = _nodes[route[0]]
					self.startMovebase()
					self.search_dist = 1.25

				elif len(goals)==1:
				# else:
					self.goal = goals[0]
					self.startMovebase()
					self.search_dist = 1.25


				elif len(goals)==0:
					if self.search_dist < min_search_dist:
						rospy.loginfo("EXPLORATOION COMPLETE")
						sys.exit()

					else:
						rospy.loginfo("Decaying search radius to %f",self.search_dist)		
						self.search_dist = self.search_dist-0.25
					
				#----------------------------------MOVE_BASE_CLIENT CALL--------------------------------------#

				#----------------------------------------------------------------------------------------------#
			elif self.map_status == "waiting":
				print "MAP not available "
	







	def movebase_client(self):
		client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
		# Waits until the action server has started up and started listening for goals.
		client.wait_for_server()
		# Creates a new goal with the MoveBaseGoal constructor
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x= self.goal[0]
		goal.target_pose.pose.position.y = self.goal[1]
		self.goal_list.append(self.goal)
		goal.target_pose.pose.orientation.x= 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = 0.0
		goal.target_pose.pose.orientation.w = 1.0
		client.send_goal(goal)
		wait = client.wait_for_result( timeout = rospy.Duration(60))
		return wait


def startExploration():

	sample_obj.explorationPlanner()

if __name__ == '__main__':
	rospy.init_node('Exploration_module')
	sample_obj = explore()
	startExploration()
 


