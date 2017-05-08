#!/usr/bin/env python
"""Capture synchronized data from ROS Gazebo  """
__author__ =  'nasma1'
__version__=  '1.0'
__license__ = 'BSD'

# Python libs
import sys, time, copy

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import rospy

# Ros Messages
import message_filters
from sensor_msgs.msg import Image, CameraInfo

# Display messages
VERBOSE = True
VER = "alpha"


class bcolors:
    """ Color-encoding chars for color terminal output 
    
        Note:
            Do not forget to print bcolors.ENDC at the end of the line
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    INVERSE = '\033[96m'

def printv(str):
    """ Verpose printer
    
        Args:
            str (str): String to be printed
    """
    if VERBOSE:
        print str
    
def write(top, str, PIC):
    """ Writes a string on the image
    
        Args:
            top (int): Distance between the top edge of the image and the 
                       baseline of the text
            str (str): String to be writen
            
            PIC (np.array): 2D array of uint8 numbers representing the picture
    """    
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(PIC,str,(10,top), font, .4,(0,0,0),4)
    cv2.putText(PIC,str,(10,top), font, .4,(255,255,255),1)

class data_getter:
    """ Main class of the cpaturing process """
    
    time_slope = 0.1 # max difference in time between synchronized frames
    
    height = 320    # height and
    width = 240     # width of the camera image (as well as the depth image)
    
    # class private variables
    _callback_counter = -1
    _PIC = None
    _EverythingIsAwesome = False
    
    def __init__(self):
        '''Initialize synced ROS subscriber'''
        printv(bcolors.HEADER + "Initializing ..." + bcolors.ENDC)

        # Subscriber Topic -- they are synchronized!
        self.sub_image_L = message_filters.Subscriber("/visual/camera_L/image_raw", Image, queue_size = 1)
        self.sub_image_R = message_filters.Subscriber("/visual/camera_R/image_raw", Image, queue_size = 1)
        self.sub_kinect_img = message_filters.Subscriber("/kinect/rgb/image_raw", Image, queue_size = 1)
        self.sub_kinect_depth = message_filters.Subscriber("/kinect/depth/image_raw", Image, queue_size = 1)
        
        # define listeners' sync frame
        listeners = [self.sub_image_L, self.sub_image_R, self.sub_kinect_img, self.sub_kinect_depth]
        
	# register the synced callback
        ats = message_filters.ApproximateTimeSynchronizer(listeners, 1, self.time_slope)
        ats.registerCallback(self.image_callback)
    
        printv(bcolors.OKGREEN + bcolors.BOLD + "subscribed!" + bcolors.ENDC)
            
    def show(self):
        """ Function that just displays the camera images """
        while not rospy.is_shutdown():
            if type(self._PIC) != type(None):
                cv2.imshow('image', self._PIC)
                cv2.waitKey(1000/25) #25 fps
        
  

    def image_callback(self, img_data_L, img_data_R, img_kinect, depth_kinect):        
        """ Make one image from four different sensory data, label and align
            them into a two-times bigger image
            
            Args:
                img_data_L (np.array, uint8): left camera image (width*height*3) - RGB
                img_data_R (np.array, uint8): right camera image (width*height*3) - RGB
                img_kinect (np.array, uint8): kinect camera image (width*height*3) - RGB
                depth_kinect (np.array, float32): kinect depth sensor image (width*height) - distances in meters
                 
            Note:
                The function has no return value, it sets the self._PIC variable
        """
        PIC_L = self.img_callback(img_data_L)
        PIC_R = self.img_callback(img_data_R)
        KIN_I = self.img_callback(img_kinect)        
        KIN_D = self.kin_callback(depth_kinect)
        
        write(18,"LEFT",PIC_L)
        write(18,"RIGHT",PIC_R)
        write(18,"KINECT_COLOR",KIN_I)
        write(18,"KINECT_DEPTH",KIN_D)
                
        ss1 = np.hstack((PIC_L, PIC_R))
        ss2 = np.hstack((KIN_I, KIN_D))
        
        sidebyside = np.vstack((ss1,ss2))
        
        self._PIC = sidebyside

    def kin_callback(self, ros_data):
        """ Processes kindect depth image and returns a HSV heatmap-colored
            image representing the depth values
        
            Args:
                ros_data (str): raw sensory data originating from the
                appropriate rostopic
            
            Return:
                width*height*3 size np.array containing uint8 values (image)
                
            Note:
                The coloring is not consistent! It means that the scale is
                from the closest object to the farthest, which means that in
                different images the objects 1 meter from the sensor may appear
                in different colors.
        """
        
        npArray = np.fromstring(ros_data.data, np.float32)

        DEPTH = npArray.reshape(self.width,self.height)
               
        (min, max) = (np.nanmin(DEPTH[:]), np.nanmax(DEPTH[:]))

	# calculate the depth range from min and max measured distance
        dyn = max-min*1.0
        
        # color the depth image
        import colorsys
        N = 255*3+110
        HSV_tuples = [(x*1.0/N, 1, 1) for x in range(N)]
        RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
        
        heatmap = (np.array(RGB_tuples)*255).astype(np.uint8)
        heatmap[0] = (127,127,127)        
        
        DEPTH = DEPTH - min*1.0
        DEPTH = DEPTH/dyn
        DEPTH = DEPTH * 255*3

        DEPTH = np.nan_to_num(DEPTH).astype(np.uint16)
        
        NANS = heatmap[DEPTH]
        
        return NANS
    
    def img_callback(self, ros_data):
        """ Callback function of subscribed image topic
            
            Args:
                ros_data (str): raw sensory data originating from the
                appropriate rostopic
            
            Retrun:
                width*height*3 size np.array containing uint8 values (image)
        """
        
        npArray = np.fromstring(ros_data.data, np.uint8)
        
        if self.width*self.height*3 == npArray.size :        
            PIC = npArray.reshape(self.width,self.height,3)            
        else :
            PIC = None                       
        return PIC
           
def main(args):
    """ Initializes and cleans up the ros node """
    
    rospy.init_node('cam_capture', anonymous=False)
    rosnode = data_getter()
   
    rosnode.show()
            
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
