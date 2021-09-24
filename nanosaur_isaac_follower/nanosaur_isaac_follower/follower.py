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

class Follower(Node):
    
    def __init__(self):
        super().__init__('nanosaur_follower')
        # Get ID to follow
        self.declare_parameter("id", 4)
        self.april_tag_id = self.get_parameter("id").value
        #Init QoS
        qos_profile = QoSProfile(depth=5)
        # Create command Twist publisher
        self.pub_nav_ = self.create_publisher(Twist, 'nav_vel', qos_profile)
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
        self.get_logger().info("Hello Follower!")

    def move_eyes(self, center):
        eyes_msg = Eyes()
        # Convert center
        eyes_msg.x = -2 * (center.x - 160)/ 320.0 * 100.0
        eyes_msg.y = (center.y - 160) / 240.0 * 100.0
        self.get_logger().info(f"[{eyes_msg.x:.0f}, {eyes_msg.y:.0f}]")
        # Wrap to Eyes message
        self.pub_eyes_.publish(eyes_msg)

    def drive_robot(self, center):
        twist = Twist()
        # Convert center
        twist.angular.z = -0.1 * (center.x - 160)/ 320.0 * 100.0
        # Wrap to Eyes message
        self.pub_nav_.publish(twist)

    def april_tag(self, msg):
        if msg.detections:
            for detect in msg.detections:
                if detect.family == '36h11':
                    center = detect.center
                    # If Detect the april tag, enable follow
                    if detect.id == self.april_tag_id:
                        self.get_logger().info(f"DETECTED! [{center.x:.2f}, {center.y:.2f}]")
                        self.move_eyes(center)
                        self.drive_robot(center)
                        break
                    else:
                        self.get_logger().info(f"ID {detect.id} [{center.x:.2f}, {center.y:.2f}]")
        #self.get_logger().info(f"{msg}")

def main(args=None):
    rclpy.init(args=args)
    # Init AprilTag follower node
    follower = Follower()
    # Start node
    try:
        rclpy.spin(follower)
    except (KeyboardInterrupt, SystemExit):
        pass
    # Destroy the node explicitly
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# EOF
