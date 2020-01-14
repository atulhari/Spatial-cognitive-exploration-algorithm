from polygon_Mapper_gp import polygonMapper
import rospy
def start():
    global x1,x1f
    s_object = polygonMapper()
    while not rospy.is_shutdown():
    	do = True
    	polygonMapper.scanToPolygonConvertor(s_object,do)
    	do = True
    	rospy.sleep(1) 



if __name__ == '__main__':
    # while not rospy.is_shutdown():
    start()
        # rospy.spin()
 

