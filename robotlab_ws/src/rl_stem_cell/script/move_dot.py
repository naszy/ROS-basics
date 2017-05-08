#!/usr/bin/env python
""" Move red_dot in ROS Gazebo  """

__author__ =  'nasma1'
__version__=  '0.1'
__license__ = 'BSD'

# Python libs
import sys, time, copy

# numpy and scipy
import numpy as np

# Ros libraries
import roslib
import rospy

# Ros Messages
from geometry_msgs.msg import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *


class talker:

    def __init__(self):        
        # rosservice
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

               
    def move(self):
        # random coordinates
        x = np.random.random_integers(-10,10)
        y = np.random.random_integers(-10,10)
        z = np.random.random_integers(-10,10)
        
        p_new = Point(x,y,z)
        pose = Pose(position=p_new)
        model_state = ModelState(model_name="robot", pose=pose, reference_frame="world")

        self.set_model_state(model_state)
         
           
def main(args):
    T = talker()
    rospy.init_node('talker', anonymous=True)
    T.move()

if __name__ == '__main__':
    main(sys.argv)
