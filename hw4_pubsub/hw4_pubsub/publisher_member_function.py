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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geomtry
from zed_interfaces.msg import ObjectsStamped

class PID:
    """PID (Proportional-Integral-Derivative) controller.

    Used in control systems to correct the error between 
    a measured process variable and a desired setpoint by 
    calculating and then outputting a corrective action 
    that can adjust the process accordingly.
    
    Parameters
    ----------
    Kp : float
    Proportional gain

    Ti : float
    Integral time constant

    Td : float 
    Derivative time constant 

    dt : float
    Time step
    """
    def __init__(self, Kp, Ti, Td, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt
    

    def update_control(self, current_error, reset_prev=False):
        # todo: implement this
        # For each incoming laser scan,
        # issue an angular velocity command to the robot at the topic /husky/cmd vel, based on the output of the PID
        # controller. You need to follow the wall on the LEFT of the robot, as it is placed in its initial configuration.

        # The PID controller should be implemented as a discrete-time controller.

        if reset_prev:
            # reset error accumulation so it doesnt grow too big in the Integral portion
            self.sum_error = 0.0
   
        
        if self.dt == 0 or self.Ti == 0:
            # prevent division by zero in derivative calculation
            self.control = 0.0
            return self.control
        
        # update errors
        self.prev_error_deriv = self.curr_error_deriv
        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.sum_error += self.curr_error
        self.curr_error_deriv = (self.curr_error - self.prev_error) / self.dt

        # PID controller formula: 
        # Kp * error + Ki * integral(error) + Kd * derivative(error).
        # self.Kp, self.Ti, and self.Td are the proportional, integral, and derivative gains respectively.
        P = self.Kp * self.curr_error 
        I = self.Kp * 1/self.Ti * self.sum_error
        D = self.Kp * self.Td * self.curr_error_deriv
        self.control = P + I + D
        
        return self.control
      

    def get_control(self):
        return self.control

class ObjectSubscriber(Node):

    def __init__(self):
        super().__init__('object_subscriber')
        self.zed_subscriber = self.create_subscription(String, 'zed_subscriber', 10)
        self.pid_controller = PID(
            Kp=self.Kp.value, 
            Ti=self.Ti.value, 
            Td=self.Td.value, 
            dt=1/self.hz)
        # TODO: change the path as the robot is not husky
        self.cmd_pub = self.create_publisher(Twist, '/husky/cmd_vel', QoSProfile(depth=10))

    def object_callback(self, msg):
        """Get objects from Zed2 camera"""
        objects = msg.objects
        for obj in objects:
            # assume object has class and position
            obj_class = obj.object_class
            # assume position.x: distance to the object
            obj_distance = obj.position.x
            self.get_logger().info('Object class: %s, distance: %f' % (obj_class, obj_distance))
    
    def move_callback(self):
        """Move the robot"""
        pass

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