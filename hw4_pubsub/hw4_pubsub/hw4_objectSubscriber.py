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
        self.count = 0


    def object_callback(self, msg):
        """Get objects from Zed2 camera and move the robot"""
        twist = Twist()
        objects = msg.objects
        if objects:
            for obj in objects:
                # check if object is near enough, move
                if math.sqrt(obj.position.x**2 + obj.position.y**2) < self.desired_distance_from_object:
                    twist.linear.x = self.forward_speed
                    print(f"move {self.count}")
                    break
            else:
                print(f"halt {self.count}")
                twist.linear.x = 0.0
        else:
            # move anyway
            print(f"move anyway {self.count}")
            twist.linear.x = self.forward_speed
        # publish the message
        self.cmd_pub.publish(twist)
        self.count += 1
      

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
