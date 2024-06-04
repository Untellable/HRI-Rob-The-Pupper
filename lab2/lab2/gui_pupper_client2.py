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
from pupper_interfaces.srv import GoPupper
import readchar
# Import for the touch sensors
import time
import RPi.GPIO as GPIO
import cv2
import numpy as np

import tkinter as tk
from tkinter import messagebox, PhotoImage

# When calculating the percent of pixels matching between two images, 
# use these thresholds and labels to describe how close they are
similarity_thresholds = np.array([.6, .8, .9])
similarity_threshold_labels = ["not similar", "somewhat similar", "similar", "very similar"]
quadrant_names = ["top left", "top right", "bottom left", "bottom right"]

# mask the image quadrants
def mask_quadrants(live_image, mask_image, quad_flags = [True, True, True, True], alpha = 0.5):
    assert live_image.shape[0] == mask_image.shape[0] and live_image.shape[1] == mask_image.shape[1]
    height, width = mask_image.shape

    result = np.copy(live_image)
    mask_image /= 255

    #scale from 0-1 to alpha-1
    mask_image = (1 - alpha) * mask_image + alpha

    mask_image = np.repeat(mask_image[:, :, np.newaxis], 3, axis=2)

    for i in range(2):
        for j in range(2):
            if quad_flags[i * 2 + j]:
                result[i * height // 2: (i + 1) * height // 2, j * width // 2: (j + 1) * width // 2] *= mask_image[i * height // 2: (i + 1) * height // 2, j * width // 2: (j + 1) * width // 2]

    return result

                   
# Convert similarity percent into string descriptor based on thresholds
def threshold_perc(perc_similar):
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
    for row in [0,half_len]:
        for col in [0,half_len]:
            comp_results.append(np.mean(im1[row:row + half_len] == im2[col:col + half_len]))
    return comp_results


# Move an image 
# diff: int tuple pair on number of pixels to move horizontally and vertically.
def move_image(im, diff: tuple[int, int]):
    pad = max(abs(diff[0]), abs(diff[1]))
    # Pads with 2's which aren't in the mask, so padding will never match with any part of the mask
    padded_im = np.pad(im, pad, constant_values = 2)
    (im_x, im_y) = im.shape
    return padded_im[pad - diff[0]: pad + im_x - diff[0], pad - diff[1]: pad + im_y - diff[1]]

# Translate move (numpy array 2 int elements) into a readable text explanation
def explain_move(move, im_shape):
    move_dirs = (move > 0).astype(np.uint8)
    x_names = ["left", "right"]
    y_names = ["up", "down"]

    # Calculate distance to move as percent of image dimensions
    x_perc = np.rint(100 * abs(move[1] / im_shape[1])).astype(int)
    y_perc = np.rint(100 * abs(move[0] / im_shape[0])).astype(int)

    return f"{x_perc}% {x_names[move_dirs[0]]} and {y_perc}% {y_names[move_dirs[1]]}"
    


# Initiate ORB detector
orb = cv2.ORB_create()

# Comparing two images, look for keypoint pairs between them
# Test if moving the first image to match keypoint pairs would make any of the 
# image quadrants match. Only checks for translations, not rotations
# n is then number of keypoint pair translations to test
# NOTE: matching performs badly with ~<200x200 images
def check_close_quad(im1, im2, n = 5):
    # find the keypoints and descriptors with ORB
    # Method taken from https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html
    kp1, des1 = orb.detectAndCompute(im1,None)
    kp2, des2 = orb.detectAndCompute(im2,None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
     
    # Match descriptors.
    matches = bf.match(des1,des2)
     
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    top_score = 0
    top_quad = 0
    top_move = (0,0)
    for i in range(min(n, len(matches))):
        # Calculate keypoint distance between images for one match
        kp1Idx = matches[i].queryIdx
        kp2Idx = matches[i].trainIdx
        move = np.flip(np.rint(kp2[kp2Idx].pt).astype(int) - np.rint(kp1[kp1Idx].pt).astype(int))
        quad_scores = compare_quads(move_image(im1, move), im2)
        # Calc best quad match
        for quad in range(4):
            if quad_scores[quad] > top_score:
                top_score = quad_scores[quad]
                top_quad = quad
                top_move = move
    threshold_text = threshold_perc(top_score)
    if threshold_text == "not similar":
        return f"I didn't detect any nearby matches"
    move_text = explain_move(top_move, im1.shape)
    return f"I detect a {threshold_text} match for the {quadrant_names[top_quad]} quadrant if you move {move_text}."

# Convert RGB image to binary mask, with slight blurring to reduce noise
# Range of color for mask creation defaulting to red from lab1
def camera_to_mask(rgb_im, color_lower = [164, 60, 100], color_upper = [179, 205, 255]):
	# Convert the current frame from RGB to HSV
    hsv = cv2.cvtColor(rgb_im, cv2.COLOR_BGR2HSV)

    # Create numpy arrays from the boundaries
    color_lower = np.array(color_lower,dtype=np.uint8)
    color_upper = np.array(color_upper,dtype=np.uint8)   
        
    color_mask = cv2.inRange(hsv,color_lower,color_upper)
    blurred = cv2.blur(color_mask, (20,20))
    _, bw = cv2.threshold(blurred,127,255,cv2.THRESH_OTSU)
    return bw

# Compare RGB image to mask image
def compare_to_mask(rgb_im, mask):
    camera_mask = camera_to_mask(rgb_im)
    return check_close_quad(camera_mask, mask)

# Takes RGB image, converts it to a mask, and saved at the specified location
def save_new_mask(rgb_im, save_path):
    camera_mask = camera_to_mask(rgb_im)
    cv2.imwrite(save_path, camera_mask)

def start_puzzle():
    if not level.get():
        messagebox.showinfo("Select Level", "Please select a difficulty level!")
        return

    global feedback_text
    # Create a new window
    puzzle_window = tk.Toplevel(root)
    puzzle_window.title(f"Puzzle {level.get()}")
    print('hi')
    # Image mask (use an actual image file path in the same folder)
    image_path = "/home/ubuntu/HRI-Rob-The-Pupper/lab2/lab2/1-1.png"  # Ensure 'your_image.png' is in the same folder as your script
    # image_path = f"mask_{level.get().lower()}.png"
    img = PhotoImage(file=image_path)
    image_label = tk.Label(puzzle_window, image=img)
    image_label.image = img  # Keep a reference!
    image_label.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # Feedback text box
    feedback_text = tk.Text(puzzle_window, height=10, width=30)
    feedback_text.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
    # feedback_text.insert(tk.END, "Feedback will appear here.")
    im1 = cv2.imread("/home/ubuntu/HRI-Rob-The-Pupper/lab2/lab2/1-1.png", cv2.IMREAD_UNCHANGED)
    im2 = cv2.imread("/home/ubuntu/HRI-Rob-The-Pupper/lab2/lab2/1-1_moved.png", cv2.IMREAD_UNCHANGED)
    text_feedback = check_close_quad(im1, im2)
    update_feedback(text_feedback)


def update_feedback(message):
    """Update the feedback text box with a new message."""
    if feedback_text:
        feedback_text.delete('1.0', tk.END)  # Clear existing text
        feedback_text.insert(tk.END, message)  # Insert new message

def set_level(selected_level):
    level.set(selected_level)
    # Reset all buttons to RAISED
    easy_button.config(relief=tk.RAISED)
    medium_button.config(relief=tk.RAISED)
    hard_button.config(relief=tk.RAISED)
    # Set the selected button to SUNKEN
    if selected_level == "Easy":
        easy_button.config(relief=tk.SUNKEN)
    elif selected_level == "Medium":
        medium_button.config(relief=tk.SUNKEN)
    elif selected_level == "Hard":
        hard_button.config(relief=tk.SUNKEN)

def quit_application():
    global minimal_client
    minimal_client.destroy_node()
    rclpy.shutdown()
    root.destroy()


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
    root.mainloop()

    # Destroy node and shut down
    minimal_client.destroy_node()
    rclpy.shutdown()

# Main window
root = tk.Tk()
root.title("Rob the Builder")

# Heading
heading_label = tk.Label(root, text="Rob the Builder", font=("Arial", 24))
heading_label.pack(pady=20)

# Difficulty level options
level = tk.StringVar()  # To store the selected difficulty level
frame = tk.Frame(root)
frame.pack(pady=20)

easy_button = tk.Button(frame, text="Easy", command=lambda: set_level("Easy"))
easy_button.pack(side=tk.LEFT)

medium_button = tk.Button(frame, text="Medium", command=lambda: set_level("Medium"))
medium_button.pack(side=tk.LEFT)

hard_button = tk.Button(frame, text="Hard", command=lambda: set_level("Hard"))
hard_button.pack(side=tk.LEFT)

# Start button
start_button = tk.Button(root, text="Start the Puzzle", command=start_puzzle)
start_button.pack(pady=20)

# Quit button
quit_button = tk.Button(root, text="Quit", command=quit_application)
quit_button.pack(side=tk.RIGHT, pady=20, padx=10)

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
    if display_string == '':
        display_string = 'No button touched'
        imgLoc = '/home/ubuntu/tmp_dir/img_dir/stop_new.jpg'

    # Display corresponding image in pupper
    disp.show_image(imgLoc)

    # Send the move request to the service
    minimal_client.send_move_request(display_string)

root.bind("<Key>", key_pressed_ros)    

if __name__ == '__main__':
    main()

