#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from geometry_msgs.msg import PoseStamped
from scipy.spatial import KDTree, cKDTree,distance
from visualization_msgs.msg import MarkerArray,Marker


c_samples = [0.40,0.66,0.19]
s_samples = [0.2,0.2,0.2]
c_small_samples = [0.9,0.67,0.0]
s_small_samples = [0.3,0.3,0.3]
class OdomToPath:
    def __init__(self):
        # self.path_pub = rospy.Publisher('/path', Path, latch=True, queue_size=10)

        # self.odom_sub = rospy.Subscriber('/robot_pose', PoseStamped, self.odom_cb, queue_size=1)
        self.tr_pub = rospy.Publisher('trajectory_node', MarkerArray, queue_size=10)
        self.path = Path()

    # def odom_cb(self, msg):

    #     cur_posex = msg.pose.position.x
    #     cur_posey = msg.pose.position.y
    #     self.current_position = (cur_posex,cur_posey)


    def publishMarker(self,msg,c,s):
            count = 0
            MARKERS_MAX = 100
            markerArray = MarkerArray()
            # frontiers = poly2
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

            if(count > MARKERS_MAX):
                markerArray.markers.pop(0)

            # markerArray.markers.append(markers)
                # Renumber the marker IDs
            id = 0

            for m in markerArray.markers:
                m.id = id
                id += 1
            return markerArray

    def path_pub(self):
        trajectory_node = (0.0,0.0)
        trajectory_resolution = 0.1
        self.trajectory = []
        self.total_path = 0.0

        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/robot_pose",PoseStamped)
            
        # self.path.header = msg.header
            self.current_position = (msg.pose.position.x,msg.pose.position.y)
            # print self.current_position
            self.trajectory.append(self.current_position)
            self.path_length = distance.euclidean(self.current_position,trajectory_node)
            if distance.euclidean(self.current_position,trajectory_node)>trajectory_resolution:
                self.trajectory.append(self.current_position)
                trajectory_node = self.current_position
                self.total_path +=self.path_length  
            # self.path.poses.append(cur_pose)
            if len(self.trajectory)>1:
                trajectory_node_marker = self.publishMarker(self.trajectory,c_samples,s_samples)
                self.tr_pub.publish(trajectory_node_marker)
                # pub.publish(self.trajectory)
            rospy.sleep(2)
            rospy.loginfo("Trajectory length: %s",self.total_path)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('odom_to_path')
    odom_to_path = OdomToPath()
    odom_to_path.path_pub()
    rospy.spin()