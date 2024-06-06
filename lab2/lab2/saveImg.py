import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import time

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.image_count = 0
        self.current_time = time.time()

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Save the image
            img_filename = f'/home/ubuntu/transfer_dir/camera_image.png'
            if self.current_time - time.time() > 0.5:           
                cv2.imwrite(img_filename, cv_image)
                self.current_time = time.time()
                self.get_logger().info(f'Saved image {img_filename}')
            self.image_count += 1
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
        
def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaverNode()
    try:
        rclpy.spin(image_saver_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        image_saver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
