#Filename: turtle_sensor.py
#Students: Ben Klingensmith, Niyas Attasseri, Anand Kumar
#Lab 1, controlling turtle using colors
#
#Description: This creates a publisher to detect singificant presence of blue 
# or red in the camera feed. It tells the turtle to move if blue is primarily 
# detected, and to stop if red is primarily detected.
#
#How to use:
#Run camera: ros2 launch depthai_ros_driver camera.launch.py
#Run turtlesim: ros2 run turtlesim turtlesim_node
#Run publisher: ros2 run lab2 turtle_sensor

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import os

THRESHOLD = .1
STOP_TWIST = Twist()
MOVE_TWIST = Twist()
MOVE_TWIST.linear.x = .5

# Creating a class for the echo camera node. Note that this class inherits from the Node class.
class echo_camera(Node):
    def __init__(self):
        #Initializing a node with the name 'echo_camera'
        super().__init__('echo_camera')
        
        #Subscribing to the /oak/rgb/image_raw topic that carries data of Image type
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.echo_topic, 10)
        self.subscription #this is just to remove unused variable warnings
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        #CvBridge has functions that allow you to convert ROS Image type data into OpenCV images
        self.br = CvBridge()

        #Initialize color boundaries for red and blue:
        self.red_lower = np.array([164, 60, 100])
        self.red_upper = np.array([179, 205, 255])
        
        self.blue_lower = np.array([85, 50, 165])
        self.blue_upper = np.array([105, 200, 255])  
        
        # Define current state, which can be 'stop' or 'move'
        self.cur_state = "stop"
        self.n_frame = 0  
        print(os.getcwd())
        
        self.root = 'images/'
        os.makedirs(self.root, exist_ok=True)
    
    # Callback function to echo the video frame being received  
    def echo_topic(self, data):
        #Logging a message - helps with debugging later on
        self.get_logger().info('Receiving video frame')
        
        #Using the CvBridge function imgmsg_to_cv to convert ROS Image to OpenCV image. Now you can use this image to do other OpenCV things
        current_frame = self.br.imgmsg_to_cv2(data)
        
        
        # Convert the current frame from RGB to HSV
        cv2.imwrite(self.root+str(self.n_frame)+'.png',current_frame)
        cv2.imshow('frame', current_frame)
        cv2.waitKey(1)
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Create numpy arrays from the boundaries
        red_lower = np.array(self.red_lower,dtype=np.uint8)
        red_upper = np.array(self.red_upper,dtype=np.uint8)
        blue_lower = np.array(self.blue_lower,dtype=np.uint8)
        blue_upper = np.array(self.blue_upper,dtype=np.uint8)
        
        
        red_mask = cv2.inRange(hsv,red_lower,red_upper)
        blue_mask = cv2.inRange(hsv,blue_lower,blue_upper)
        
        
        # Track percentage of image which is red or blue
        red_perc = np.mean(red_mask)
        blue_perc = np.mean(blue_mask)
        
        # Choose twist to publish based on current state and detected color
        cur_twist = None
        if self.cur_state == "stop":
            # Start moving if blue detected
            if blue_perc > THRESHOLD and blue_perc > red_perc and self.shape_detect_from_mask(blue_mask):
                self.cur_state = "move"
                cur_twist = MOVE_TWIST
            else:
                cur_twist = STOP_TWIST
        elif self.cur_state == "move":
            # Stop moving if red detected
            if red_perc > THRESHOLD and red_perc > blue_perc and self.shape_detect_from_mask(red_mask):
                self.cur_state = "stop"
                cur_twist = STOP_TWIST
            else:
                cur_twist = MOVE_TWIST
                
        # Send twist update to turtle
        #self.publisher_.publish(cur_twist)
        
    def shape_detect_from_mask(self, mask):
        '''
        Function to detect if the mask is rectangular

        Input:
	        Mask: mask array of size (w, h)

        Output:
	        Boolean: True if there is an rectangle
        '''

        #blur and threshold the mask
        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        
        #find the contours
        cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #iterate over the contours and detect any rectangle
        for c in cnts:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            
            if len(approx) == 4:
            	#is a rectangle
                return True
        
        return False

        

# Main function         
def main(args=None):
    # Initializing rclpy (ROS Client Library for Python)
    rclpy.init(args=args)
    
    #Create an object of the echo_camera class
    echo_obj = echo_camera()
    
    #Keep going till termination
    rclpy.spin(echo_obj)
    
    #Destroy node when done 
    echo_obj.destroy_node()
    
    #Shutdown rclpy
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
