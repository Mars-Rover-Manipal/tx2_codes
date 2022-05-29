#!/usr/bin/python3
import sys
import numpy as np
import pyzed.sl as sl
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import signal
import sys
import time
import threading


class zed_api_class():

    def __init__(self):

        # Initialise ROS Node
        rospy.init_node('zed_node')

        # Create a ZED camera object
        self.zed = sl.Camera()

        self.key = ' '
        self.help_string = "[s] Save side by side image [d] Save Depth, [n] Change Depth format, [p] Save Point Cloud, [m] Change Point Cloud format, [q] Quit"
        self.prefix_point_cloud = "Cloud_"
        self.prefix_depth = "Depth_"
        self.path = "./"
        self.count_save = 0
        self.mode_point_cloud = 0
        self.mode_depth = 0
        self.point_cloud_format_ext = ".ply"
        self.depth_format_ext = ".png"
        self.image_message = Image()
        self.image_size = self.zed.get_camera_information().camera_resolution
        self.image_ocv_r_channel = []
        self.image_ocv_g_channel = []
        self.image_ocv_b_channel = []
        self.image_ocv = sl.Mat(self.image_size.width,self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.image_publish = np.zeros((1080, 1920, 3), dtype = "uint8")
                            
                               

        # Set configuration parameters
        self.input_type = sl.InputType()
        if len(sys.argv) >= 2:
            self.input_type.set_from_svo_file(sys.argv[1])
        self.init = sl.InitParameters(input_t=self.input_type)
        self.init.camera_resolution = sl.RESOLUTION.HD1080
        self.init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.init.coordinate_units = sl.UNIT.MILLIMETER

        # Open the camera
        self.err = self.zed.open(self.init)
        if self.err != sl.ERROR_CODE.SUCCESS:
            print(repr(self.err))
            self.zed.close()
            exit(1)

        # Display help in console
        print("Started ZED API. Getting Image from Camera...")

        # Set runtime parameters after opening the camera
        self.runtime = sl.RuntimeParameters()
        self.runtime.sensing_mode = sl.SENSING_MODE.STANDARD

        # Prepare new image size to retrieve half-resolution images
        self.image_size.width = self.image_size.width / 2
        self.image_size.height = self.image_size.height / 2

        # Declare your sl.Mat matrices
        self.image_zed = sl.Mat(self.image_size.width,
                                self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.depth_image_zed = sl.Mat(
            self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.point_cloud = sl.Mat()

        # Thread Defenitions :
        self.th_convert_to_ROS_msg = threading.Thread(
            target=self.convert_to_ROS_msg)

        # Publisher Defenitions
        self.image_pub = rospy.Publisher(
            "/Zed2/image_raw", Image, queue_size=10)

        # cv_bridge defenitions
        self.bridge = CvBridge()

        # Call Main
        self.get_image()
        
    def process_key_event(self, zed, key):

        if key == 100 or key == 68:
            save_depth(zed, path + prefix_depth + str(self.count_save))
            self.count_save += 1
        elif key == 110 or key == 78:
            self.mode_depth += 1
            self.depth_format_ext = depth_format_name()
            print("Depth format: ", self.depth_format_ext)
        elif key == 112 or key == 80:
            self.save_point_cloud(
                self.zed, self.path + self.prefix_point_cloud + str(self.count_save))
            self.count_save += 1
        elif key == 109 or key == 77:
            self.mode_point_cloud += 1
            self.point_cloud_format_ext = self.point_cloud_format_name()
            print("Point Cloud format: ", self.point_cloud_format_ext)
        elif self.key == 104 or key == 72:
            print(help_string)
        elif key == 115:
            self.save_sbs_image(zed, "ZED_image" +
                                str(self.count_save) + ".png")
            self.count_save += 1
        else:
            a = 0
            
    def convert_to_ROS_msg(self):
        self.image_message = self.bridge.cv2_to_imgmsg(
            self.image_ocv, "passthrough")
        self.image_pub.publish(image_message)

    def get_image(self):
        self.key = ''
        while self.key != 113:
            try:
                self.err = self.zed.grab(self.runtime)
                if self.err == sl.ERROR_CODE.SUCCESS:

                    # Retrieve the left image, depth image in the half-resolution
                    self.zed.retrieve_image(
                        self.image_zed, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
                    #self.zed.retrieve_image(self.depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, self.image_size)

                    # Retrieve the RGBA point cloud in half resolution
                    #self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, self.image_size)

                    # To recover data from sl.Mat to use it with opencv, use the get_data() method
                    # It returns a numpy array that can be used as a matrix with opencv
                    self.image_ocv = self.image_zed.get_data()
                    #self.depth_image_ocv = self.depth_image_zed.get_data()
                    self.image_ocv = cv2.cvtColor( self.image_ocv, cv2.COLOR_BGRA2BGR )
                    self.image_ocv_r_channel = self.image_ocv[:,:,2]
                    self.image_ocv_g_channel = self.image_ocv[:,:,0]
                    self.image_ocv_b_channel = self.image_ocv[:,:,1]
                    self.image_publish[:,:,0] = self.image_ocv_b_channel
                    self.image_publish[:,:,1] = self.image_ocv_g_channel
                    self.image_publish[:,:,2] = self.image_ocv_r_channel
                    cv2.cvtColor(self.image_publish, cv2.COLOR_RGB2BGR )
                    rospy.loginfo(self.image_publish.shape)
                    
                    self.image_message = self.bridge.cv2_to_imgmsg(self.image_publish, "passthrough")
                    self.image_message.encoding = "bgr8"
                    self.image_pub.publish(self.image_message)
                    #cv2.imshow("Image", self.image_ocv)
                    #cv2.imshow("Depth", self.depth_image_ocv)
                    self.key = cv2.waitKey(10)
                    self.process_key_event(self.zed, self.key)

            except rospy.ROSInterruptException or KeyboardInterrupt:
                cv2.destroyAllWindows()
                self.zed.close()
                print("\nFINISH")
                break


if __name__ == "__main__":
    obj = zed_api_class()
