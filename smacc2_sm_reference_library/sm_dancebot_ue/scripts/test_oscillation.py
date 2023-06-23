import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener
from tf2_ros import Buffer
from tf2_ros import TransformStamped
import math

class RotationOscillation(Node):

    def __init__(self):
        super().__init__('rotation_oscillation')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def oscillate_rotation(self):
        rate = self.create_rate(1)
        angle = math.radians(25)
        timeout =  rclpy.duration.Duration(seconds=1.0)  # Timeout for waiting on transforms
        base_link_frame = 'base_footprint'
        fixed_frame = 'odom'

        print('Waiting for transform from "fixed_frame" to "base_link_frame"...')
        while not self.tf_buffer.can_transform(base_link_frame, fixed_frame, rclpy.time.Time(), timeout):
            self.get_logger().info('Waiting for transform from "fixed_frame" to "base_link_frame"...')
            rclpy.spin_once(self)
            rate.sleep()

        self.get_logger().info('Transform available!')

        # Get initial position
        print('Getting initial position...')
        try:
            base_link_transform = self.tf_buffer.lookup_transform('map', base_link_frame, rclpy.time.Time())
            initial_position = base_link_transform.transform.translation
        except Exception as e:
            self.get_logger().error('Error getting initial position: %s' % str(e))
            return

        # Rotate 25 degrees to the left
        print('Rotating 25 degrees to the left...')
        self.get_logger().info('Rotating 25 degrees to the left...')
        twist_msg = Twist()
        twist_msg.angular.z = angle
        self.cmd_vel_pub.publish(twist_msg)
        rclpy.spin_once(self)

        # Get position after left rotation
        print('Getting position after left rotation...')
        try:
            print('Getting position after left rotation...')
            base_link_transform = self.tf_buffer.lookup_transform('map', base_link_frame, rclpy.time.Time())
            left_rotation_position = base_link_transform.transform.translation
        except Exception as e:
            self.get_logger().error('Error getting position after left rotation: %s' % str(e))
            return

        # Rotate 25 degrees to the right
        print('Rotating 25 degrees to the right...')
        self.get_logger().info('Rotating 25 degrees to the right...')
        twist_msg.angular.z = -angle
        self.cmd_vel_pub.publish(twist_msg)
        rclpy.spin_once(self)

        # Get position after right rotation
        try:
            base_link_transform = self.tf_buffer.lookup_transform('map', base_link_frame, rclpy.time.Time())
            right_rotation_position = base_link_transform.transform.translation
        except Exception as e:
            self.get_logger().error('Error getting position after right rotation: %s' % str(e))
            return

        # Calculate phase difference
        print('Calculating phase difference...')
        left_rotation_angle = math.atan2(left_rotation_position.y - initial_position.y,
                                         left_rotation_position.x - initial_position.x)
        right_rotation_angle = math.atan2(right_rotation_position.y - initial_position.y,
                                          right_rotation_position.x - initial_position.x)
        phase_difference = math.degrees(right_rotation_angle - left_rotation_angle)

        self.get_logger().info('Phase difference: %f degrees' % phase_difference)

        # Stop the robot
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def shutdown(self):
        self.get_logger().info('Shutting down...')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    rotation_oscillation = RotationOscillation()

    try:
        rotation_oscillation.oscillate_rotation()
    except Exception as e:
        rotation_oscillation.get_logger().error('Error during rotation oscillation: %s' % str(e))

    rotation_oscillation.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()