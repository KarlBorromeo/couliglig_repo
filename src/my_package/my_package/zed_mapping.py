import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from  sensor_msgs.msg import LaserScan
import cv2
from message_filters import ApproximateTimeSynchronizer
import numpy as np

class ZED2DMapper(Node):
    def __init__(self):
        super().__init__('zed_2d_mapper')
        
        self.sub = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
        10)

        self.pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

    def depth_callback(self, msg):
        pass


    

def main(args=None):
    rclpy.init(args=args)
    node = ZED2DMapper()
    rclpy.spin(node)  # Keep the node alive to keep receiving images
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
