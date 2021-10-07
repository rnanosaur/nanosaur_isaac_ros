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

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nanosaur_msgs.msg import Eyes
from geometry_msgs.msg import Twist
from simple_pid import PID

if os.getenv("ISAAC_ROS_APRILTAG", 'True') == 'True':
    from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
else:
    from apriltag_msgs.msg import AprilTagDetectionArray

class Follower(Node):
    
    def __init__(self):
        super().__init__('nanosaur_follower')
        self.isaac_ros = os.getenv("ISAAC_ROS_APRILTAG", 'True') == 'True'
        # Get ID to follow
        self.declare_parameter("id", 4)
        self.april_tag_id = self.get_parameter("id").value
        # Get frame size to follow
        self.declare_parameter("frame.width", 320.0)
        self.frame_width = self.get_parameter("frame.width").value
        self.declare_parameter("frame.height", 240.0)
        self.frame_height = self.get_parameter("frame.height").value
        # Gain eyes message
        self.declare_parameter("gain.eyes.x", 1)
        self.gain_eyes_x = self.get_parameter("gain.eyes.x").value
        self.declare_parameter("gain.eyes.y", 1)
        self.gain_eyes_y = self.get_parameter("gain.eyes.y").value
        # Gain control motor
        self.declare_parameter("gain.twist.Kp", 0.012)
        twist_Kp = self.get_parameter("gain.twist.Kp").value
        self.declare_parameter("gain.twist.Ki", 0.00)
        twist_Ki = self.get_parameter("gain.twist.Ki").value
        self.declare_parameter("gain.twist.Kd", 0.000)
        twist_Kd = self.get_parameter("gain.twist.Kd").value
        self.pid_twist = PID(Kp=twist_Kp, Ki=twist_Ki, Kd=twist_Kd, output_limits=(-0.6, 0.6))
        # Linear follower gains and init PID
        self.declare_parameter("gain.linear.Kp", 0.005) # OLD 0.0003
        linear_Kp = self.get_parameter("gain.linear.Kp").value
        self.declare_parameter("gain.linear.Ki", 0.0)
        linear_Ki = self.get_parameter("gain.linear.Ki").value
        self.declare_parameter("gain.linear.Kd", 0.0)
        linear_Kd = self.get_parameter("gain.linear.Kd").value
        self.pid_linear = PID(Kp=linear_Kp, Ki=linear_Ki, Kd=linear_Kd, output_limits=(-0.05, 0.05))
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
        self.get_logger().info(f"- Linear KP:{linear_Kp} KI:{linear_Ki} KD:{linear_Kd}")
        self.get_logger().info(f"- Twist KP:{twist_Kp} KI:{twist_Ki} KD:{twist_Kd}")
        if not self.isaac_ros:
            self.get_logger().info("APRILTAG CPU!!!")

    def follower_stop(self):
        self.pub_nav_.publish(Twist())
        self.pub_eyes_.publish(Eyes())

    def move_eyes(self, error_x, error_y):
        eyes_msg = Eyes()
        # Convert center to eye movement
        eyes_msg.x = self.gain_eyes_x * error_x
        eyes_msg.y = self.gain_eyes_y * error_y
        # self.get_logger().info(f"[{eyes_msg.x:.0f}, {eyes_msg.y:.0f}]")
        # Wrap to Eyes message
        self.pub_eyes_.publish(eyes_msg)

    def size_tag(self, corners):
        x0 = corners[0].x
        y0 = corners[0].y
        x1 = corners[1].x
        y1 = corners[1].y
        return ((((x1 - x0)**2) + ((y1 - y0)**2))**0.5)

    def detect_tag(self, detections):
        tag = {"detect": False, "px": 0., "py": 0., "size": 0.}
        for detect in detections:
            if detect.family == '36h11' if self.isaac_ros else 'tag36h11':
                # If detect QR code
                if detect.id == self.april_tag_id:
                    tag["detect"] = True
                    # Center tag
                    center = detect.center if self.isaac_ros else detect.centre
                    tag["px"] = - 200.0 * (center.x - (self.frame_width / 2.)) / self.frame_width
                    tag["py"] = 200.0 * (center.y - (self.frame_height / 2.)) / self.frame_height
                    # Size tag
                    tag["size"] = self.size_tag(detect.corners)
                    break
        return tag

    def april_tag(self, msg):
        tag = self.detect_tag(msg.detections)
        detect = "Y" if tag["detect"] else "N"
        px = tag["px"]
        py = tag["py"]
        size = tag["size"]
        # test = ex / (size / 50.) if size > 0 else 0
        self.pid_linear.setpoint = 180. if tag["detect"] else 0.
        #self.get_logger().info(f"ID {self.april_tag_id} - Ex: [{px:.1f} - {py:.1f}] - Size: {size:.1f}")
        # Send Drive message
        twist = Twist()
        lin_control = self.pid_linear(size)
        twist.linear.x = lin_control if lin_control > 0. else 0.
        twist.angular.z = -self.pid_twist(px)
        # self.get_logger().info(f"Detect: {detect} - {size:.2f} | lin: {twist.linear.x:.4f} - ste: {twist.angular.z:.4f}")
        isaac = "GPU" if self.isaac_ros else "CPU"
        self.get_logger().info(f"{isaac} Detect: {detect} | S: {size:.2f} Ex [{px:.0f} ,{py:.0f}]")
        self.pub_nav_.publish(twist)
        # Update eyes
        self.move_eyes(px, py)

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
