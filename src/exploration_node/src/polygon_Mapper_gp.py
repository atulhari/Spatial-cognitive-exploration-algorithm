#!/usr/bin/env python

# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PolygonStamped, Point32
import matplotlib.pyplot as plt
import matplotlib
from visualization_msgs.msg import MarkerArray,Marker
import numpy as np
# from math import cos,sin,ceil
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from shapely.geometry import Polygon, Point, LineString, LinearRing
from shapely.ops import cascaded_union
from shapely import speedups
from simplification.cutil import simplify_coords, simplify_coords_vw
from std_msgs.msg import String
from scipy.spatial import KDTree, cKDTree,distance

# from nav_msgs.msg import OccupancyGrid
# import cv2
# from copy import copy


speedups.enable()

# Parameters

current_area_ = 0.0 #initializing area under visible map 
area_scale_ = 0.05 # Builds map every time area increases by the scale 
poly_buffer_ = -0.25 # Buffer applied on new polygon 
poly_init_buffer_ = -0.25 #Buffer applied on initial polygon: -0.008 
min_voxel_width_ = 0.8 # min width of the frontier that should be detected
simplification_factor = 0.04 # How much sparse the PCL should be | Line simplicfication
# mapData = OccupancyGrid()



class polygonMapper(object):
	def __init__(self):
		# self.map2  = OccupancyGrid()
		self.sub_scan_map = rospy.Subscriber('/PointCloud2Map', PointCloud2, self.callBackPcl, queue_size=100)
		self.poly_pub = rospy.Publisher("polygon_map", PolygonStamped, queue_size=50)
		self.frontier_pub = rospy.Publisher('frontier_segment', MarkerArray, queue_size=10)
		self.status_pub = rospy.Publisher('map_status', String, queue_size=10)



	def callBackPcl(self,sub_scan_map):
	    
	    self.map_points = [point for point in pc2.read_points(sub_scan_map, skip_nans=True)]


	def cascadedUnion(self,*geoms):
		
		return cascaded_union([geom if geom.is_valid else geom.buffer(0.0) for geom in geoms])


	def extractPoints(self,point,bool):

		# tolerance = (len(self.map_points)*0.008)
		self.mapxy = [(points[0],points[1]) for points in self.map_points ]
		rospy.loginfo("Incoming points from the cloud: %s", len(self.mapxy))
		simplified = simplify_coords(np.vstack(self.mapxy),simplification_factor)
		return simplified



	def rgb(self,minimum, maximum, value):
		minimum, maximum = float(minimum), float(maximum)
		ratio = 2 * (value-minimum) / (maximum - minimum)
		b = int(max(0, 255*(1 - ratio)))
		r = int(max(0, 255*(ratio - 1)))
		g = 255 - b - r
		return r, g, b

	def publishMarker(self,msg,cell_val):
		count = 0
		MARKERS_MAX = 100
		markerArray = MarkerArray()

		val = 0

		for p in (msg):
			x = 2*cell_val[val]
			r,g,b = self.rgb(1,30,x)
			val = val + 1
			z = np.log(x)
			markers = Marker()
			markers.header.frame_id = '/map'
			markers.header.stamp = rospy.Time.now()
			markers.ns = "frontier_segment"
			markers.action = markers.ADD
			markers.type = markers.SPHERE
			# markers.id = 0

			markers.scale.x = 0.1
			markers.scale.y = 0.1
			markers.scale.z = 0.1


			markers.color.a = 1.0
			markers.color.r = r
			markers.color.g = g
			markers.color.b = b

			markers.pose.position.x = p[0]
			markers.pose.position.y = p[1]
			markers.pose.position.z = -0.2 #z
			markerArray.markers.append(markers)

		if(count > MARKERS_MAX):
			markerArray.markers.pop(0)

		# markerArray.markers.append(markers)
			# Renumber the marker IDs
		id = 0

		for m in markerArray.markers:
			m.id = id
			id += 1
		return markerArray  	


	def scanToPolygonConvertor(self,run_init):
		# 
		rospy.init_node('Polygon_map_generator')  

		# rate = rospy.Rate(10) # 10hz
		self.status = "waiting"
		self.status_pub.publish(self.status)
		loop = 1
		if run_init == True:
			points_xy_init = self.extractPoints(self.map_points,False)	
			poly_init = Polygon(points_xy_init).buffer(poly_init_buffer_)
			rospy.loginfo("Polygon map initiated with area: %s", poly_init.area)
			run_init = False
		self.updated_map = []

		for loop in range(0,1) : #and poly_init.area - current_area_ > 0.2:
			points_xy = self.extractPoints(self.map_points,True)	
			poly = Polygon(points_xy).buffer(poly_buffer_)
	
			rospy.loginfo("Extracted points from cloud: %s", len(points_xy))
			self.updated_map = self.cascadedUnion(poly,poly_init)
			rospy.loginfo("map merged")

			if self.updated_map.type == 'MultiPolygon':
				self.updated_map =  max(self.updated_map, key=lambda a: a.area)

			rospy.loginfo("Polygon mapped area: %s", poly_init.area)
			poly_init = self.updated_map.buffer(poly_init_buffer_)

			rospy.loginfo("ready to publish")			

			polygonMap_ = PolygonStamped()
			polygonMap_.header.stamp = rospy.Time.now()
			polygonMap_.header.frame_id = '/map'
			for i in range(len(self.updated_map.exterior.coords)):
				polygonMap_.polygon.points.append(Point32(x=self.updated_map.exterior.coords[i][0], y=self.updated_map.exterior.coords[i][1]))
			
			self.poly_pub.publish(polygonMap_)
			rospy.loginfo("map updated and published")
			loop = loop + 1
			self.status = "done"
			self.status_pub.publish(self.status)
			rospy.loginfo("Polygon map status: %s", self.status)

	

