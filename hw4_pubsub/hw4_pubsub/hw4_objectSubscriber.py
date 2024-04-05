# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from zed_interfaces.msg import ObjectsStamped



class ObjectSubscriber(Node):

    def __init__(self):
        super().__init__('object_subscriber')
        qos = QoSProfile(depth=10)
        # get subscription to zed camera
        self.zed_subscriber = self.create_subscription(ObjectsStamped, '/zed/zed_node/obj_det/objects', self.object_callback, qos)
        self.desired_distance_from_object = 0.1
        self.forward_speed = 0.1
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
        print("we good maboi")


    def object_callback(self, msg):
        """Get objects from Zed2 camera and move the robot"""
        print("ya girl")
        objects = msg.objects
        distances = [math.sqrt(obj.position[0] **2 + obj.position[1] **2) for obj in objects]
        # create a Twist message
        twist = Twist()
        if any(distance < self.desired_distance_from_object for distance in distances):
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.forward_speed
        self.cmd_pub.publish(twist)
       
        # # if distance > 0.1m: move forward
        # if obj_distance > self.desired_distance_from_object:
        #     # set the linear velocity
        #     twist.linear.x = self.forward_speed
        #     # publish the message
        #     self.cmd_pub.publish(twist)
        # else:
        #     # stop the robot
        #     twist.linear.x = 0.0
        #     self.cmd_pub.publish(twist)
      

def main(args=None):
    rclpy.init(args=args)

    object_subscriber = ObjectSubscriber()
    rclpy.spin(object_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# /zed/zed_node/obj_det/objects
