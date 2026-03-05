#!/usr/bin/env python
import rospy
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time


puente = CvBridge()
imagen = Image()
def cb_camera(msg):
    global imagen
    # Muestra la imagen 
    print("hola")
    imag = puente.imgmsg_to_cv2(msg)
    imagen=cv2.cvtColor(imag,cv2.COLOR_BGR2RGB)

def cb_3d_lidar(msg):
    pass

rospy.init_node("Nodo")

rospy.Subscriber("/robot/front_rgbd_camera/rgb/image_raw",Image,cb_camera)
rospy.Subscriber("/robot/top_3d_laser/rslidar_points",PointCloud2,cb_3d_lidar)

pub = rospy.Publisher("/robot/cmd_vel",Twist,queue_size=10)

a = Twist()
rate = rospy.Rate(4)
time.sleep(3)
while not rospy.is_shutdown():


    if (cv2.waitKey(1) == ord('s')):
        break
    
    
    cv2.imshow('webCam',imagen)

    a.linear.x = 0
    a.angular.z = 0
    pub.publish(a)

    rate.sleep()

cv2.destroyAllWindows()
