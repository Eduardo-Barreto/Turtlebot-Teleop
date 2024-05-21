import rclpy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
import sys
import termios
import tty
import select


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key


def main():
    rclpy.init()
    node = rclpy.create_node("teleop_keys")

    linear_publisher = node.create_publisher(Float32, "linear_speed", 10)
    angular_publisher = node.create_publisher(Float32, "angular_speed", 10)
    kill_client = node.create_client(Trigger, "kill_robot_service")

    print(
        "Use 'w' to move forward, 's' to move backward, 'a' to turn left, 'd' to turn right, 'q' to quit"
    )

    while rclpy.ok():
        linear_speed = Float32()
        angular_speed = Float32()
        key = get_key()

        if key == "q":
            break

        elif key in ("w", "s"):
            linear_speed.data = 70.0 if key == "w" else -70.0

        elif key in ("a", "d"):
            angular_speed.data = 70.0 if key == "a" else -70.0

        linear_publisher.publish(linear_speed)
        angular_publisher.publish(angular_speed)
        print(
            f"\rLinear speed: {linear_speed.data} | Angular speed: {angular_speed.data}"
        )

    request = Trigger.Request()
    kill_client.call_async(request)
    node.destroy_node()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))


if __name__ == "__main__":
    main()
