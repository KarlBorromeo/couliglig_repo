from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from controller import Robot, Supervisor, InertialUnit
from cv_bridge import CvBridge
import rclpy
import numpy as np
import math

HALF_DISTANCE_BETWEEN_WHEELS = 0.24 / 2
WHEEL_RADIUS = 0.05

class RobotDriver:
    def init(self, webots_node, properties):
       
        rclpy.init(args=None)
        self.__robot: Robot = webots_node.robot
        self.__supervisor: Supervisor = webots_node.robot.getSelf()
        self.__time_step = int(self.__robot.getBasicTimeStep())
        
        self.__trans_field = self.__supervisor.getField("translation")
        self.__rotation_field = self.__supervisor.getField("rotation")
        self.__inertial_unit: InertialUnit = self.__robot.getDevice('inertial_unit')
        self.__inertial_unit.enable(self.__time_step)

        robot_name = str(self.__robot.getName())
        self.__robot_name = robot_name

        self.__node = rclpy.create_node(robot_name)

        self.__node.get_logger().info('  - properties: ' + str(properties))
        self.__node.get_logger().info('  - robot name: ' + robot_name)
        self.__node.get_logger().info('  - basic timestep: ' + str(int(self.__robot.getBasicTimeStep())))

        self.__node.get_logger().info('  - is supervisor? ' + str(self.__robot.getSupervisor()))

        self.__node.create_subscription(Clock, 'clock', self.__clock_callback, 1)
        
        self.__clock = Clock()
        
        self.__setup_motors()

        self.__target_twist = Twist()

        self.__publisher = self.__node.create_publisher(Clock, '~/custom_clock' if robot_name[-1].isdigit() else 'custom_clock', 1)
        cmd_vel_topic = '~/cmd_vel' if robot_name[-1].isdigit() else 'cmd_vel'
        odom_topic = '~/odom' if robot_name[-1].isdigit() else 'odom'
        tf_topic = '~/tf' if robot_name[-1].isdigit() else 'tf'

        self.__node.create_subscription(Twist, cmd_vel_topic, self.__cmd_vel_callback, 1)
        self.__odom_publisher = self.__node.create_publisher(Odometry, odom_topic, 10)
        self.__tf_pub = self.__node.create_publisher(TFMessage, tf_topic, 10)

        self.__node.create_timer(1.0 / self.__time_step, self.__publish_odom)

        self.__cv_bridge = CvBridge()

    def __setup_motors(self):
        self.__left_motor = self.__robot.getDevice('left_motor')
        self.__right_motor = self.__robot.getDevice('right_motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)      

    def __clock_callback(self, msg):
        self.__clock = msg

    def __cmd_vel_callback(self, msg):
        self.__target_twist = msg

    def __imu_callback(self, msg):
        self.__imu_data = msg

    def __get_rotation(self):
        rotation = self.__rotation_field.getSFRotation()
        x_axis, y_axis, z_axis, angle = rotation

        half_angle = angle / 2.0
        sin_half_angle = math.sin(half_angle)

        qx = x_axis * sin_half_angle
        qy = y_axis * sin_half_angle
        qz = z_axis * sin_half_angle
        qw = math.cos(half_angle)

        # Normalize the quaternion to avoid TF errors
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm > 0.0:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

        return qx, qy, qz, qw

    def __publish_odom(self):
        trans_values = self.__trans_field.getSFVec3f()

        ori_values_quat = self.__inertial_unit.getQuaternion()
        ori_values_quat_rnd = np.array(ori_values_quat)

        x_pos, y_pos, z_pos = (trans_values[0], trans_values[1], trans_values[2])

        # x_ori, y_ori, z_ori, w_ori = (
        #     ori_values_quat_rnd[0],
        #     ori_values_quat_rnd[1],
        #     ori_values_quat_rnd[2],
        #     ori_values_quat_rnd[3],
        # )
        x_ori, y_ori, z_ori, w_ori = self.__get_rotation()

        # header_frame_id = "/" + self.__robot_name + "/odom" if  self.__robot_name[-1].isdigit() else "odom"
        # child_frame_id = "/" + self.__robot_name + "/base_link" if  self.__robot_name[-1].isdigit() else "base_link"

        odom_msg = Odometry()
        odom_msg.header.stamp = self.__clock.clock
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = x_pos
        odom_msg.pose.pose.position.y = y_pos
        odom_msg.pose.pose.position.z = z_pos
        odom_msg.pose.pose.orientation.x = x_ori
        odom_msg.pose.pose.orientation.y = y_ori
        odom_msg.pose.pose.orientation.z = z_ori
        odom_msg.pose.pose.orientation.w = w_ori

        self.__odom_publisher.publish(odom_msg)

        t = TransformStamped()

        t.header.stamp = self.__clock.clock
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = x_pos
        t.transform.translation.y = y_pos
        t.transform.translation.z = z_pos

        t.transform.rotation.x = x_ori
        t.transform.rotation.y = y_ori
        t.transform.rotation.z = z_ori
        t.transform.rotation.w = w_ori

        # self.__broadcaster.sendTransform(t)
        self.__tf_pub.publish(TFMessage(transforms=[t]))


    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

        self.__publisher.publish(self.__clock)
    