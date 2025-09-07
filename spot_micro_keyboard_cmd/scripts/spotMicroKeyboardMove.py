#!/usr/bin/env python3

"""
Class for sending keyboard commands to spot micro walk node, control velocity, yaw rate, and walk event
Ported to ROS 2 with rclpy
"""
import rclpy
from rclpy.node import Node
import sys, select, termios, tty  # For terminal keyboard key press reading
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from math import pi

msg = """
Spot Micro Walk Command

Enter one of the following options:
-----------------------------
quit: stop and quit the program
walk: Start walk mode and keyboard motion control
stand: Stand robot up
idle: Lay robot down
angle_cmd: enter angle control mode

Keyboard commands for body motion
---------------------------
   q   w   e            u
   a   s   d

  u: Quit body motion command mode and go back to rest mode
  w: Increment forward speed command / decrease pitch angle
  a: Increment left speed command / left roll angle
  s: Increment backward speed command / increase pitch angle
  d: Increment right speed command / right roll angle
  q: Increment body yaw rate command / left yaw angle (negative left, positive right)
  e: Increment body yaw rate command / right yaw angle (negative left, positive right)
  f: In walk mode, zero out all rate commands.

  anything else: Prompt again for command

CTRL-C to quit
"""

valid_cmds = ('quit', 'Quit', 'walk', 'stand', 'idle', 'angle_cmd')

# Global body motion increment values
speed_inc = 0.02
yaw_rate_inc = 3 * pi / 180
angle_inc = 2.5 * pi / 180

class SpotMicroKeyboardControl(Node):
    def __init__(self):
        super().__init__('spot_micro_keyboard')

        # Initialize message objects
        self._angle_cmd_msg = Vector3()
        self._angle_cmd_msg.x = 0.0
        self._angle_cmd_msg.y = 0.0
        self._angle_cmd_msg.z = 0.0

        self._vel_cmd_msg = Twist()
        self._vel_cmd_msg.linear.x = 0.0
        self._vel_cmd_msg.linear.y = 0.0
        self._vel_cmd_msg.linear.z = 0.0
        self._vel_cmd_msg.angular.x = 0.0
        self._vel_cmd_msg.angular.y = 0.0
        self._vel_cmd_msg.angular.z = 0.0

        self._walk_event_cmd_msg = Bool()
        self._walk_event_cmd_msg.data = True

        self._stand_event_cmd_msg = Bool()
        self._stand_event_cmd_msg.data = True

        self._idle_event_cmd_msg = Bool()
        self._idle_event_cmd_msg.data = True

        # Publishers
        self._ros_pub_vel_cmd = self.create_publisher(Twist, '/vel_cmd', 10)
        self._ros_pub_angle_cmd = self.create_publisher(Vector3, '/angle_cmd', 10)
        self._ros_pub_walk_cmd = self.create_publisher(Bool, '/walk_cmd', 10)
        self._ros_pub_stand_cmd = self.create_publisher(Bool, '/stand_cmd', 10)
        self._ros_pub_idle_cmd = self.create_publisher(Bool, '/idle_cmd', 10)

        self.get_logger().info("Setting Up the Spot Micro Keyboard Control Node...")
        print(msg)

    def get_key(self):
        # Non-blocking keyboard input
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        while rclpy.ok():
            userInput = self.get_key()
            if userInput:
                if userInput in valid_cmds:
                    if userInput in ('quit', 'Quit'):
                        self.get_logger().info('Quitting...')
                        break
                    elif userInput == 'walk':
                        self._walk_event_cmd_msg.data = True
                        self._ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                        self.get_logger().info('Command issued to start walk mode.')
                    elif userInput == 'stand':
                        self._stand_event_cmd_msg.data = True
                        self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                        self.get_logger().info('Command issued to stand.')
                    elif userInput == 'idle':
                        self._idle_event_cmd_msg.data = True
                        self._ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)
                        self.get_logger().info('Command issued to idle.')
                    elif userInput == 'angle_cmd':
                        self.get_logger().info('Entering angle control mode...')
                        # Angle control loop
                        while rclpy.ok():
                            key = self.get_key()
                            if key == 'u':
                                self.get_logger().info('Exiting angle control mode.')
                                break
                            elif key == 'w':
                                self._angle_cmd_msg.y -= angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                            elif key == 's':
                                self._angle_cmd_msg.y += angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                            elif key == 'a':
                                self._angle_cmd_msg.x -= angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                            elif key == 'd':
                                self._angle_cmd_msg.x += angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                            elif key == 'q':
                                self._angle_cmd_msg.z -= angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                            elif key == 'e':
                                self._angle_cmd_msg.z += angle_inc
                                self._ros_pub_angle_cmd.publish(self._angle_cmd_msg)
                            else:
                                self.get_logger().warn(f'Invalid angle command issued: {key}')
                            rclpy.spin_once(self, timeout_sec=0.1)
                elif userInput in ('w', 'a', 's', 'd', 'q', 'e', 'u', 'f'):
                    if userInput == 'u':
                        self.get_logger().info('Quit body motion command mode and go back to rest mode.')
                        break
                    elif userInput == 'w':
                        self._vel_cmd_msg.linear.x += speed_inc
                        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                    elif userInput == 's':
                        self._vel_cmd_msg.linear.x -= speed_inc
                        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                    elif userInput == 'a':
                        self._vel_cmd_msg.linear.y -= speed_inc
                        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                    elif userInput == 'd':
                        self._vel_cmd_msg.linear.y += speed_inc
                        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                    elif userInput == 'q':
                        self._vel_cmd_msg.angular.z -= yaw_rate_inc
                        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                    elif userInput == 'e':
                        self._vel_cmd_msg.angular.z += yaw_rate_inc
                        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                    elif userInput == 'f':
                        self._vel_cmd_msg.linear.x = 0.0
                        self._vel_cmd_msg.linear.y = 0.0
                        self._vel_cmd_msg.angular.z = 0.0
                        self._ros_pub_vel_cmd.publish(self._vel_cmd_msg)
                        self.get_logger().info('Command issued to zero all rate commands.')
                else:
                    print('Key not in valid key commands, try again')
                    self.get_logger().warn(f'Invalid keyboard command issued in walk mode: {userInput}')
                rclpy.spin_once(self, timeout_sec=0.1)

if __name__ == "__main__":
    rclpy.init()
    smkc = SpotMicroKeyboardControl()
    try:
        smkc.run()
    except KeyboardInterrupt:
        smkc.get_logger().info('Shutting down due to CTRL-C.')
    finally:
        rclpy.shutdown()
