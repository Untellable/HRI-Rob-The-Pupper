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

# Import for the touch sensors
import time
import RPi.GPIO as GPIO

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
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()

    # debug - comment in/our as needed
    # print("In client, got this command: %s" % cmd)

    # This spins up a client node, checks if it's done, throws an exception of there's an issue
    # (Probably a bit redundant with other code and can be simplified. But right now it works, so ¯\_(ツ)_/¯)
    # Feedback loop
    while rclpy.ok():

        # Obtain input from touch sensors
        touchValue_Front = GPIO.input(touchPin_Front)
        touchValue_Back = GPIO.input(touchPin_Back)
        touchValue_Left = GPIO.input(touchPin_Left)
        touchValue_Right = GPIO.input(touchPin_Right)
        display_string = ''

        # if else chain for changing the state of the pupper based on the touch sensor
        if not touchValue_Front:
            display_string += 'move_forward'
            imgLoc = '/home/ubuntu/tmp_dir/img_dir/front_new.jpg'
        elif not touchValue_Right:
            # display_string += 'move_right'
            display_string += 'look_up'
            imgLoc = '/home/ubuntu/tmp_dir/img_dir/right_new.jpg'
        elif not touchValue_Left:
            # display_string += 'move_left'
            display_string += 'look_down'
            imgLoc = '/home/ubuntu/tmp_dir/img_dir/left_new.jpg'
        if display_string == '':
            display_string = 'No button touched'
            imgLoc = '/home/ubuntu/tmp_dir/img_dir/stop_new.jpg'
        print(display_string)

        # Display corresponding image in pupper
        disp.show_image(imgLoc)

        # Send the move request to the service
        minimal_client.send_move_request(display_string)

        # get response and print for each loop
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of command: %s ' %
                    (response))
            # break

    # Destroy node and shut down
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
