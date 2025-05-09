import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from zed_msgs.msg import ObjectsStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
from message_filters import ApproximateTimeSynchronizer
import numpy as np

class ZEDImageSubscriber(Node):
    def __init__(self):
        super().__init__('my_camera')
        self.bridge = CvBridge()
        self.detected_objects = []
        # Subscribe to the RGB image topic from the ZED camera
        self.img_sub = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.img_listener_callback,
        10)
        self.det_sub = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.det_listener_callback,
        10)

    def det_listener_callback(self,msg: ObjectsStamped):
        for obj in msg.objects:
            self.detected_objects.append(
                {
                    "label": obj.label,
                    "corners" : np.array([[kp.kp[0]//2, kp.kp[1]//2] for kp in obj.bounding_box_2d.corners], dtype=np.int32),
                }
            )

    def img_listener_callback(self, msg: Image):
        try:
            # self.get_logger().info(f"Image encoding: {msg.encoding}")
            if msg.height == 0 or msg.width == 0:
                self.get_logger().warn('Received empty image message')
                return
           
            cv_image = self.bridge.imgmsg_to_cv2(msg) 
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR) 

            # print(self.detected_objects)

            for obj in self.detected_objects:
                # print(obj['corners'])
                cv2.polylines(cv_image, [obj['corners']], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.putText(cv_image, obj['label'], (obj['corners'][0][0], obj['corners'][0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)           

            cv2.imshow("ZED Image", cv_image)
            self.detected_objects = []
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"General Error: {str(e)}")

    

def main(args=None):
    rclpy.init(args=args)
    node = ZEDImageSubscriber()
    rclpy.spin(node)  # Keep the node alive to keep receiving images
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
