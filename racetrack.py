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
from zed_interfaces.msg import ObjectsStamped, Object
import math


class ExtendedObject():
    def __init__(self, parent_object):
        self.parent = parent_object
        self.min_distance = None
    def compute_min_distance(self):
        self.min_distance = math.sqrt(self.parent.position[0]**2 + self.parent.position[1]**2)

class ObjectSubscriber(Node):

    def __init__(self):
        super().__init__('object_subscriber')
        qos = QoSProfile(depth=10)
        # get subscription to zed camera
        self.zed_subscriber = self.create_subscription(ObjectsStamped, '/zed/zed_node/obj_det/objects', self.object_callback, qos)
        self.desired_distance_from_object = 0.4
        self.forward_speed = 0.05
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
        self.count = 0
        print("we good maboi")


    def object_callback(self, msg):
        """Get objects from Zed2 camera and move the robot"""
        # print(f"count {self.count}")
        objects = [ExtendedObject(obj) for obj in msg.objects]
        # create a Twist message

        # FOR EVASIVE MANEUVERS:
        # if object is too close, set angular velocity to (some value),
            # angular velocity can be calculated from distance from obstacle & thiccness of robot
        # else, use PID controls
            # how to get the 'error' for PID to minimise?
                # from map plan a path and follow?
                # then get deviation from path as error

        twist = Twist()
        if objects:
            # calculate min distance for all objects and print labels
            for i in range(len(objects)):
                objects[i].compute_min_distance()
                print(f"Object Class: {objects[i].label}, Distance: {math.sqrt(objects[i].min_distance)}, Coordinates: {objects[i].parent.position[0], objects[i].parent.position[1]}")
            # get the nearest object, set angle based on siam distance
            nearest_obj = min(objects, key=lambda obj: obj.min_distance)

            # decide whether to keep going forward
            min_distance = nearest_obj.min_distance
            if min_distance < self.desired_distance_from_object:
                twist.linear.x= 0.0
                print(f"reverse {self.count}")
            elif min_distance < 1.0:
                twist.linear.x = -self.forward_speed*2
            else:
                twist.linear.x = self.forward_speed 
                print(f"go {self.count}")
                # if its too near, move back.
                twist.angular.z = self.set_angle(nearest_obj)
        else:
            twist.linear.x = self.forward_speed
            print("no objects")
        self.cmd_pub.publish(twist)
        self.count += 1

    def set_angle(self, closest_object):
        robot_radius = 0.2
        obstacle_radius = 0.2
        siam_distance = robot_radius + obstacle_radius
        # check if closest object is too close on the right and left
        if abs(closest_object.parent.position[1]) < siam_distance:
            # take evasive action
            # TODO: smoothen out with PID controller

            # forward distance, check if y is too close.
            x = closest_object.parent.position[0]
            ratio = siam_distance/x
            if (ratio <= 1 and ratio >= -1):
                goRight = 1 if (closest_object.parent.position[0] > 0) else -1
                return math.asin(ratio) * goRight
            else:
                # robot is way too close, either back up or turn fully left/right
                # backing up is simpler
                print("backing up")
                return 0.0

        else:
            # dont change current angle
            return 0.0
      

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
