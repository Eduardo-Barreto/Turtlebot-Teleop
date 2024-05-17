import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

MAX_LINEAR_VEL = 0.22
MAX_ANGULAR_VEL = 2.84


def map_value(x, in_min, in_max, out_min, out_max):
    """
    Function to map a value from one range to another.

    Args:
        x (float): Value to be mapped.
        in_min (float): Minimum value of the input range.
        in_max (float): Maximum value of the input range.
        out_min (float): Minimum value of the output range.
        out_max (float): Maximum value of the output range.
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class TeleopService(Node):
    def __init__(self):
        """
        Initialize the teleop service node.
        """
        super().__init__("teleop_service")

        self.current_speed = Twist()
        self.last_speed = None

        self.linear_subscriber = self.create_subscription(
            Float32, "linear_speed", self.linear_callback, 10
        )
        self.angular_subscriber = self.create_subscription(
            Float32, "angular_speed", self.angular_callback, 10
        )

        self.is_killed = False
        self.kill_service = self.create_service(Empty, "kill_robot", self.kill_callback)

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Teleop service node initialized.")

    def kill_callback(self, request, response):
        """
        Callback to terminate the teleop service node.
        """
        self.get_logger().info("Teleop service node terminated.")
        self.is_killed = True
        return response

    def linear_callback(self, msg):
        """
        Callback to receive linear velocity.
        """
        linear_speed = map_value(msg.data, -100, 100, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
        self.current_speed.linear.x = linear_speed

    def angular_callback(self, msg):
        """
        Callback to receive angular velocity.
        """
        angular_speed = map_value(
            msg.data, -100, 100, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL
        )
        self.current_speed.angular.z = angular_speed

    def spin(self):
        """
        Runs the teleop service node.
        """
        try:
            while rclpy.ok() and not self.is_killed:
                if self.current_speed != self.last_speed:
                    self.get_logger().info(
                        f"Linear speed: {self.current_speed.linear.x} | Angular speed: {self.current_speed.angular.z}"
                    )
                    self.cmd_vel_publisher.publish(self.current_speed)
                    self.last_speed = self.current_speed

                rclpy.spin_once(self)

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

        finally:
            self.stop_robot()
            self.get_logger().info("Teleop service node terminated.")

    def stop_robot(self):
        """
        Stops the robot's movement.
        """
        self.current_speed = Twist()
        self.cmd_vel_publisher.publish(self.current_speed)
        self.get_logger().info("Robot movement stopped.")


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopService()
    teleop_node.spin()
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
