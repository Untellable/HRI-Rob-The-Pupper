########
# Author: Anand Kumar, Ben Klingensmith, Niyas Attasseri
# Name: client_go_pupper
#
# Purpose: Lab 2 Task 5. Client for the pupper to move based on the user's touch and change display based on the state.
#
# Usage: First launch the service (see lab/file). Then you can run the client like this:
#        ros2 run lab2task5 client
#
#
# Date: 07 May 2024
########

# Import the ROS2 interface we wrote, called GoPupper. This specifies the message type.
import sounddevice as sd
from pupper_interfaces.srv import GoPupper
import readchar
# Import for the touch sensors
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np
import audio2numpy as a2n


# When calculating the percent of pixels matching between two images,
# use these thresholds and labels to describe how close they are
similarity_thresholds = np.array([.6, .8, .9])
similarity_threshold_labels = ["not similar", "somewhat similar", "similar", "very similar", "matching"]

quadrant_names = ["top left", "top right", "bottom left", "bottom right"]


# Convert similarity percent into string descriptor based on thresholds
def threshold_text(perc_similar):
    thresholds_passed = np.sum(perc_similar > similarity_thresholds)
    similarity_description = similarity_threshold_labels[thresholds_passed]
    return similarity_description


# Calculate percent shared pixels for each quadrant pair between 2 images
def compare_quads(im1, im2):
    # Expects images be square and same size
    side_len = im1.shape[0]
    half_len = side_len // 2
    assert side_len == im2.shape[0]
    comp_results = []
    for row in [0, half_len]:
        for col in [0, half_len]:
            comp_results.append(np.mean(im1[row:row + half_len] == im2[col:col + half_len]))
    return comp_results


# Translate an image
# diff: int tuple pair on number of pixels to move horizontally and vertically.
def move_image(im, diff: tuple[int, int]):
    pad = max(abs(diff[0]), abs(diff[1]))
    padded_im = np.pad(im, pad, constant_values=-1)
    (im_x, im_y) = im.shape
    return padded_im[pad - diff[0]: pad + im_x - diff[0], pad - diff[1]: pad + im_y - diff[1]]


# Translate move (numpy array 2 int elements) into a readable text version
def explain_move(move, im_shape):
    move_dirs = move > 0
    x_names = ["left", "right"]
    y_names = ["up", "down"]

    # Calculate distance to move as percent of image dimensions
    x_perc = np.rint(100 * abs(move[0] / im_shape[0])).astype(int)
    y_perc = np.rint(100 * abs(move[1] / im_shape[1])).astype(int)

    return f"{x_perc}% {x_names[move_dirs[0]]} and {y_perc}% {y_names[move_dirs[1]]}"


# Initiate ORB detector
orb = cv2.ORB_create()


# Comparing two images, look for keypoint pairs between them
# Test if moving the first image to match keypoint pairs would make any of the
# image quadrants match. Only checks for translations, not rotations
# n is then number of keypoint pair translations to test
# NOTE: matching performs badly with ~<200x200 images
def check_close_quad(im1, im2, n=3):
    # find the keypoints and descriptors with ORB
    # Method taken from https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html
    kp1, des1 = orb.detectAndCompute(im1, None)
    kp2, des2 = orb.detectAndCompute(im2, None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors.
    matches = bf.match(des1, des2)

    # Sort them in the order of their distance.
    matches = sorted(matches, key=lambda x: x.distance)

    top_match_score = 0
    top_match_quad = 0
    top_match_move = (0, 0)
    for i in range(max(n, len(matches))):
        # Calculate keypoint distance between images for one match
        kp1Idx = matches[i].queryIdx
        kp2Idx = matches[i].trainIdx
        move = np.rint(kp2[kp2Idx].pt).astype(int) - np.rint(kp1[kp1Idx].pt).astype(int)
        quad_scores = compare_quads(move_image(im1, move), im2)
        # Calc best quad match
        for quad in range(4):
            if quad_scores[quad] > top_match_score:
                top_score = quad_scores[quad]
                top_quad = quad
                top_move = move
    move_text = explain_move(top_move, im1.shape)
    #return f"I detect a {threshold_perc(top_score)} match for the {quadrant_names[top_quad]} quadrant if you move {top_move}"
    return f"I detect a match for the {quadrant_names[top_quad]} quadrant if you move {top_move}"



# There are 4 areas for touch actions
# Each GPIO to each touch area
touchPin_Front = 6
touchPin_Left = 3
touchPin_Right = 16
touchPin_Back = 2

# Use GPIO number but not PIN number
GPIO.setmode(GPIO.BCM)

# Set up GPIO numbers to input
GPIO.setup(touchPin_Front, GPIO.IN)
GPIO.setup(touchPin_Left, GPIO.IN)
GPIO.setup(touchPin_Right, GPIO.IN)
GPIO.setup(touchPin_Back, GPIO.IN)

# Packages to let us create nodes and spin them up
import rclpy
from rclpy.node import Node

from MangDang.mini_pupper.display import Display, BehaviorState
from resizeimage import resizeimage  # library for image resizing
from PIL import Image, ImageDraw, ImageFont  # library for image manip.

MAX_WIDTH = 320  # max width of the LCD display

# Get access to the display so we can display things
disp = Display()

# Open the expression images and resize and save them
imgLocs = ["/home/ubuntu/tmp_dir/img_dir/front", "/home/ubuntu/tmp_dir/img_dir/left",
           "/home/ubuntu/tmp_dir/img_dir/right", "/home/ubuntu/tmp_dir/img_dir/stop"]
for path in imgLocs:
    imgLoc = path + '.jpg'
    imgFile = Image.open(imgLoc)

    # Convert to RGBA if needed
    if (imgFile.format == 'PNG'):
        if (imgFile.mode != 'RGBA'):
            imgOld = imgFile.convert("RGBA")
            imgFile = Image.new('RGBA', imgOld.size, (255, 255, 255))

    # We likely also need to resize to the pupper LCD display size (320x240).
    # Note, this is sometimes a little buggy, but you can get the idea.
    width_size = (MAX_WIDTH / float(imgFile.size[0]))
    imgFile = resizeimage.resize_width(imgFile, MAX_WIDTH)

    newFileLoc = path + '_new.jpg'  # rename as you like
    # now output it (super inefficient, but it is what it is)
    imgFile.save(newFileLoc, imgFile.format)


###
# Name: Minimal Client Async
#
# Purpose: "The MinimalClientAsync class constructor initializes the node with the name minimal_client_async. "
#          "The constructor definition creates a client with the same type and name as the service node.
#          The type and name must match for the client and service to be able to communicate."
#
######
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # super().__init__('client_go_pupper')
        self.cli = self.create_client(GoPupper, 'pup_command')

        # "The while loop in the constructor checks if a service matching the type and name of the client
        # is available once a second."
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # "Finally it creates a new request object.""
        self.req = GoPupper.Request()

    ###
    # Name: send_move_request
    # Purpose: send_move_request method, send request and spin until receive response or fail
    # Arguments:  self (reference the current class), move_command (the command we plan to send to the server)
    #####
    def send_move_request(self, move_command):
        self.req = GoPupper.Request()
        self.req.command = move_command
        print("In send_move_request, command is: %s" % self.req.command)
        self.future = self.cli.call_async(self.req)  # send the command to the server
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


###
# Name: Main
# Purpose: "Constructs a MinimalClientAsync object, sends the request using
#           the passed-in command-line arguments, and logs the results."
#####
def main(args=None):
    global minimal_client
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    key = ''
    old_time = ''
    while rclpy.ok():

        #read first line of file in /home/ubuntu/transfer_dir/key.txt
        with open('/home/ubuntu/transfer_dir/key.txt', 'r') as file:
            time = file.readline()
            key = file.readline()
        
        if time == old_time:
            continue
        if key == 'w':
            minimal_client.send_move_request('move_forward')
            img_loc = '/home/ubuntu/tmp_dir/img_dir/front_new.jpg'
        elif key == 'a':
            minimal_client.send_move_request('move_left')
            img_loc = '/home/ubuntu/tmp_dir/img_dir/left_new.jpg'
        elif key == 'd':
            minimal_client.send_move_request('move_right')
            img_loc = '/home/ubuntu/tmp_dir/img_dir/right_new.jpg'
        elif key == 's':
            minimal_client.send_move_request('move_backward')
            img_loc = '/home/ubuntu/tmp_dir/img_dir/front_new.jpg'
        elif key == 'z':
            minimal_client.send_move_request('look_up')
            img_loc = '/home/ubuntu/tmp_dir/img_dir/left_new.jpg'
        elif key == 'c':
            minimal_client.send_move_request('look_down')
            img_loc = '/home/ubuntu/tmp_dir/img_dir/left_new.jpg'
        elif key == 'x':
            minimal_client.send_move_request('look_straight')
            img_loc = '/home/ubuntu/tmp_dir/img_dir/left_new.jpg'
        elif key == 'f':
            x,sr=a2n.audio_from_file("test.mp3")
            sd.play(x, sr)
        elif key == 'q':
            break
        
        old_time = time
        disp.show_image(img_loc)

    # Destroy node and shut down
    minimal_client.destroy_node()
    rclpy.shutdown()


def key_pressed_ros(event):
    global minimal_client

    if not rclpy.ok():
        minimal_client.destroy_node()
        rclpy.shutdown()

    # Obtain input from touch sensors
    touchValue_Front = GPIO.input(touchPin_Front)
    touchValue_Back = GPIO.input(touchPin_Back)
    touchValue_Left = GPIO.input(touchPin_Left)
    touchValue_Right = GPIO.input(touchPin_Right)
    display_string = ''
    key = event.keysym

    if not touchValue_Front or key == "w":
        display_string += 'move_forward'
        imgLoc = '/home/ubuntu/tmp_dir/img_dir/front_new.jpg'
    elif not touchValue_Right or key == "d":
        display_string += 'move_right'
        # display_string += 'look_up'
        imgLoc = '/home/ubuntu/tmp_dir/img_dir/right_new.jpg'
    elif not touchValue_Left or key == "a":
        display_string += 'move_left'
        # display_string += 'look_down'
        imgLoc = '/home/ubuntu/tmp_dir/img_dir/left_new.jpg'
    elif key == "s":
        display_string += 'move_backward'
        imgLoc = '/home/ubuntu/tmp_dir/img_dir/front_new.jpg'
    elif key == "z":
        display_string += 'look_up'
        imgLoc = '/home/ubuntu/tmp_dir/img_dir/left_new.jpg'
    elif key == "c":
        display_string += 'look_down'
        imgLoc = '/home/ubuntu/tmp_dir/img_dir/left_new.jpg'
    elif key == "x":
        display_string += 'look_straight'
        imgLoc = '/home/ubuntu/tmp_dir/img_dir/left_new.jpg'
    else:
        display_string = 'No button touched'
        imgLoc = '/home/ubuntu/tmp_dir/img_dir/stop_new.jpg'

    # Display corresponding image in pupper
    disp.show_image(imgLoc)

    # Send the move request to the service
    minimal_client.send_move_request(display_string)
   

if __name__ == '__main__':
    main()

