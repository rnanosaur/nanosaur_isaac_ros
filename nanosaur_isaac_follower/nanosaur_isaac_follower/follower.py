# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nanosaur_msgs.msg import Eyes
from geometry_msgs.msg import Twist
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
# from apriltag_msgs.msg import AprilTagDetectionArray

class PID:
    
    def __init__(self, limit=0.5, Kp=0., Ki=0., Kd=0.):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limit = limit
        self.sum_error = 0.
        self.error = 0.

    def check_limit(self, var):
        if var > self.limit:
            return self.limit
        elif var < -self.limit:
            return -self.limit
        else:
            return var

    def reset(self):
        self.sum_error = 0.

    def update(self, error):
        
        self.sum_error += error
        e_i = self.check_limit(self.Ki * self.sum_error)
        
        self.error = error
        
        return self.check_limit(self.Kp * error + e_i)

class Follower(Node):
    
    def __init__(self):
        super().__init__('nanosaur_follower')
        # Get ID to follow
        self.declare_parameter("id", 4)
        self.april_tag_id = self.get_parameter("id").value
        # Get frame size to follow
        self.declare_parameter("frame.width", 320.0)
        self.frame_width = self.get_parameter("frame.width").value
        self.declare_parameter("frame.height", 240.0)
        self.frame_height = self.get_parameter("frame.height").value
        # Gain eyes message
        self.declare_parameter("gain.eyes.x", 2)
        self.gain_eyes_x = self.get_parameter("gain.eyes.x").value
        self.declare_parameter("gain.eyes.y", 2)
        self.gain_eyes_y = self.get_parameter("gain.eyes.y").value
        # Gain control motor
        self.declare_parameter("gain.control", 0.001)
        self.gain_control = self.get_parameter("gain.control").value
        self.pid_twist = PID(Kp=self.gain_control, Ki=0.0001)
        
        self.declare_parameter("gain.linear", 0.05)
        self.gain_linear = self.get_parameter("gain.linear").value
        self.pid_linear = PID(Kp=self.gain_linear)
        #Init QoS
        qos_profile = QoSProfile(depth=5)
        # Create command Twist publisher
        self.declare_parameter("navigation.topic", 'nav_vel')
        navigation_topic = self.get_parameter("navigation.topic").value
        self.pub_nav_ = self.create_publisher(Twist, navigation_topic, qos_profile)
        # Create command eyes publisher
        self.pub_eyes_ = self.create_publisher(Eyes, 'eyes', qos_profile)
        # Subscribe to AprilTag Detection message
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'tag_detections',
            self.april_tag,
            1)
        self.subscription  # prevent unused variable warning
        # Node started
        self.get_logger().info("nanosaur Isaac ROS follower!")
        self.get_logger().info(f"- Gain control: {self.gain_control}")

    def follower_stop(self):
        self.pub_nav_.publish(Twist())
        self.pub_eyes_.publish(Eyes())

    def move_eyes(self, error_x=0., error_y=0.):
        eyes_msg = Eyes()
        # Convert center to eye movement
        eyes_msg.x = self.gain_eyes_x * error_x
        eyes_msg.y = self.gain_eyes_y * error_y
        # self.get_logger().info(f"[{eyes_msg.x:.0f}, {eyes_msg.y:.0f}]")
        # Wrap to Eyes message
        self.pub_eyes_.publish(eyes_msg)

    def drive_robot(self, error_x=0, dist=0):
        twist = Twist()
        # Control motor center
        #twist.angular.z = self.gain_control * error_x + self.gain_control * dist
        twist.linear.x = self.pid_linear.update(1 / dist)
        twist.angular.z = self.pid_twist.update(error_x / dist)
        self.get_logger().info(f"lin: {twist.linear.x:.1f} - ste: {twist.angular.z:.1f}")
        # Wrap to Eyes message
        self.pub_nav_.publish(twist)

    def length_corner(self, corners):
        x0 = corners[0].x
        y0 = corners[0].y
        x1 = corners[1].x
        y1 = corners[1].y
        return ((((x1 - x0)**2) + ((y1 - y0)**2))**0.5)

    def april_tag(self, msg):
        detected = False
        if msg.detections:
            for detect in msg.detections:
                if detect.family == '36h11':
                    center = detect.center
                    dist = self.length_corner(detect.corners)
                    dist = dist / 100
                    # measure error Qr code
                    error_x = - 200.0 * (center.x - (self.frame_width / 2.)) / self.frame_width
                    error_y = 200.0 * (center.y - (self.frame_height / 2.)) / self.frame_height
                    # If Detect the april tag, enable follow
                    self.get_logger().info(f"ID {detect.id} [{error_x:.2f}, {error_y:.2f}] - dist: {dist:.2f}")
                    # If detect QR code
                    if detect.id == self.april_tag_id:
                        detected = True
                        self.move_eyes(error_x, error_y)
                        self.drive_robot(error_x, dist)
                        break
        if not detected:
            #self.get_logger().info(f"No detection")
            # Stop motors
            self.move_eyes()
            self.pid_twist.reset()
            self.pid_linear.reset()
            self.pub_nav_.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    # Init AprilTag follower node
    follower = Follower()
    # Start node
    try:
        rclpy.spin(follower)
    except (KeyboardInterrupt, SystemExit):
        pass
    follower.follower_stop()
    # Destroy the node explicitly
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# EOF
