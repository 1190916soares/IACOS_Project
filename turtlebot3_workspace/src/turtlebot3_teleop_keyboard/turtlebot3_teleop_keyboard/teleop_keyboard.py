import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def teleop_keyboard():
    rclpy.init()

    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    settings = termios.tcgetattr(sys.stdin)

    try:
        print('Teleop TurtleBot3 using keyboard. Press Ctrl+C to exit.')

        while rclpy.ok():
            key = getKey(settings)
            if key == 'w':
                twist = Twist()
                twist.linear.x = 0.2
                pub.publish(twist)
            elif key == 's':
                twist = Twist()
                twist.linear.x = -0.2
                pub.publish(twist)
            elif key == 'a':
                twist = Twist()
                twist.angular.z = 0.5
                pub.publish(twist)
            elif key == 'd':
                twist = Twist()
                twist.angular.z = -0.5
                pub.publish(twist)
            else:
                if key == '\x03':
                    break

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    teleop_keyboard()
