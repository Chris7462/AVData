import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TimeReference
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from velodyne_msgs.msg import VelodyneScan
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Vector3Stamped

class Integrate():
  def __init__(self):
    # CV bridge
    self.bridge = CvBridge()
    # Subsribers
    rospy.Subscriber("gps", NavSatFix, self.GPS_callback)
    rospy.Subscriber("gps_time", TimeReference, self.GPS_time_callback)
    rospy.Subscriber("image_front_left", Image, self.Image_FL_callback)
    rospy.Subscriber("imu", Imu, self.Imu_callback)
    rospy.Subscriber("lidar_blue_scan", VelodyneScan, self.Lidar_blue_callback)
    rospy.Subscriber("lidar_green_scan", VelodyneScan, self.Lidar_green_callback)
    rospy.Subscriber("lidar_red_scan", VelodyneScan, self.Lidar_red_callback)
    rospy.Subscriber("lidar_yellow_scan", VelodyneScan, self.Lidar_yellow_callback)
    rospy.Subscriber("pose_ground_truth", PoseStamped, self.Pose_GT_callback)
    rospy.Subscriber("pose_localize", PoseStamped, self.Pose_localize_callback)
    rospy.Subscriber("pose_raw", PoseStamped, self.Pose_raw_callback)
    rospy.Subscriber("tf", TFMessage, self.TF_callback)
    rospy.Subscriber("velocity_raw", Vector3Stamped, self.Velocity_raw_callback)
    # Publishers
    self.Image_FR_pub = rospy.Publisher('image_front_right', Image, queue_size=10)
    self.Image_RR_pub = rospy.Publisher('image_rear_right', Image, queue_size=10)
    self.Image_RL_pub = rospy.Publisher('image_rear_left', Image, queue_size=10)
    self.Image_SR_pub = rospy.Publisher('image_side_right', Image, queue_size=10)
    self.Image_SL_pub = rospy.Publisher('image_side_left', Image, queue_size=10)
  
  def GPS_callback(self,data):
    self.GPS_data = data
  
  def GPS_time_callback(self,data):
    self.GPS_time_data = data
  
  def Imu_callback(self,data):
    self.Imu_data = data
  
  def Lidar_blue_callback(self,data):
    self.Lidar_blue_data = data
  
  def Lidar_green_callback(self,data):
    self.Lidar_green_data = data
  
  def Lidar_red_callback(self,data):
    self.Lidar_red_data = data
  
  def Lidar_yellow_callback(self,data):
    self.Lidar_yellow_data = data
  
  def Pose_GT_callback(self,data):
    self.Pose_GT_data = data
  
  def Pose_localize_callback(self,data):
    self.Pose_GT_data = data
  
  def Pose_raw_callback(self,data):
    self.Pose_raw_data = data
  
  def TF_callback(self,data):
    self.TF_data = data
  
  def Velocity_raw_callback(self,data):
    self.Velocity_raw_data = data
  
  def get_name(self,stamp):
    tmp = str(stamp)
    if(int(tmp[-3]) > 4):
      tmp = str(stamp+1000)
    return tmp[0:-3]
  
  def Image_FL_callback(self, data):
    image_FL_data = data
    # Get stamp as file name
    stamp = image_FL_data.header.stamp.to_nsec()
    file_name = self.get_name(stamp)
    print(file_name)
    ''''''
    # Read images
    imgs = [cv2.imread('../Ford_Data/Sample/zip/'+fo+'/'+file_name+'.png') for fo in ['FR','RR','RL','SR','SL'] ]
    # Convert to Image msg
    imgs_msg = [self.bridge.cv2_to_imgmsg(imgs[i], "bgr8") for i in range(5)]
    # Publish to topic
    self.Image_FR_pub.publish(imgs_msg[0])
    self.Image_RR_pub.publish(imgs_msg[1])
    self.Image_RL_pub.publish(imgs_msg[2])
    self.Image_SR_pub.publish(imgs_msg[3])
    self.Image_SL_pub.publish(imgs_msg[4])
    ''''''

if __name__ == '__main__':
  rospy.init_node('step1', anonymous=True) #, disable_signals=True
  rospy.Rate(20)
  Itg = Integrate()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")



