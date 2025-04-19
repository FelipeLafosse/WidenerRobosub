#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import Float64MultiArray, String
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
import time
import threading

class MotorAUV(Node):
    def __init__(self):
        super().__init__('motor_auv_node')
        self.hat = ServoKit(channels=16)

        # Defining motors using ServoKit
        self.back_left = self.hat.servo[5] #Lower Number for forward
        self.left_vert = self.hat.servo[1]
        self.right_vert = self.hat.servo[3]
        self.back_right = self.hat.servo[6]
        self.front_right = self.hat.servo[7]
        self.front_left = self.hat.servo[10] #Lower Number for forward
        self.gripper = self.hat.servo[8]  # If you want to keep the gripper

        self.back_left.set_pulse_width_range(1000,2000)
        self.left_vert.set_pulse_width_range(1000,2000)
        self.right_vert.set_pulse_width_range(1000,2000)
        self.back_right.set_pulse_width_range(1000,2000)
        self.front_left.set_pulse_width_range(1000,2000)
        self.front_right.set_pulse_width_range(1000,2000)

        self.motors = {
            "back_left": self.back_left,
            "left_vert": self.left_vert,
            "front_left": self.front_left,
            "front_right": self.front_right,
            "right_vert": self.right_vert,
            "back_right": self.back_right,
            "gripper": self.gripper,
        }

        self.motor_sub = self.create_subscription(Float64MultiArray, 'drive_topic', self.run, 10)

    def run(self,msg):
        depth = msg.data[0]
        current_depth = msg.data[1]
        heading = msg.data[2]
        angle = msg.data[3]
        direction = msg.data[4]
        speed= msg.data[5]
        angleCorrect_speed = 0.0

        #print(current_depth,end="  ")
        #print(direction)

        if (angle) < heading:
            if(angle+3) < heading and (angle+30 > heading):
                #print('equation1')
                angleCorrect_speed = ((5/9)*(heading-angle)) + 1.6666
                #angleCorrect_speed = 7.0
            elif(angle+3) > heading:
                #print('stop1')
                angleCorrect_speed = 0.0
            else:
                #print('just turning1')
                angleCorrect_speed = 20.0
        if (angle) > heading:
            if(angle-3) > heading and (angle-30 < heading):
                #print('equation2')
                angleCorrect_speed = ((5/9)*(heading-angle)) - 1.6666
                #angleCorrect_speed = -7.0
            elif(angle-3) < heading:
                #print('stop2')
                angleCorrect_speed = 0.0
            else:
                #print('just turning2')
                angleCorrect_speed = -20.0
        #print(angleCorrect_speed)
        #self.set_throttle('front_left', (angleCorrect_speed))
        #self.set_throttle('front_right', (-angleCorrect_speed))

        if (speed + angleCorrect_speed) > 100 or ((-speed) + angleCorrect_speed) < -100:
            speed = 80

        if direction == 0:
            self.set_throttle('front_right', ((angleCorrect_speed)))
            #self.set_throttle('back_right', (angleCorrect_speed))
            self.set_throttle('front_left', ((-angleCorrect_speed)))
            #self.set_throttle('back_left', (-angleCorrect_speed))
        elif direction == 1:
            self.set_throttle('front_right', (speed+angleCorrect_speed))
            self.set_throttle('back_right', (speed+angleCorrect_speed))
            self.set_throttle('front_left', (speed-angleCorrect_speed))
            self.set_throttle('back_left', (speed-angleCorrect_speed))
        elif direction == 2:
            self.set_throttle('front_right', ((-speed)+angleCorrect_speed))
            self.set_throttle('back_right', ((-speed)+angleCorrect_speed))
            self.set_throttle('front_left', ((-speed)-angleCorrect_speed))
            self.set_throttle('back_left', ((-speed)-angleCorrect_speed))
        elif direction == 3:
            self.set_throttle('front_right', ((-speed)+angleCorrect_speed))
            self.set_throttle('back_right', ((speed)+angleCorrect_speed))
            self.set_throttle('front_left', ((speed)-angleCorrect_speed))
            self.set_throttle('back_left', ((-speed)-angleCorrect_speed))
        elif direction == 4:
            self.set_throttle('front_right', ((speed)+angleCorrect_speed))
            self.set_throttle('back_right', ((-speed)+angleCorrect_speed))
            self.set_throttle('front_left', ((-speed)-angleCorrect_speed))
            self.set_throttle('back_left', ((speed)-angleCorrect_speed))
        
        #self.depth_control(current_depth,depth)
            


    def set_throttle(self, motor_name, throttle):
        if motor_name in self.motors:
            if throttle > 0:
                if throttle > 100:
                    print('Throttle Too HIGH')    
                x = (-9/10)*throttle + 90
                self.set_angle(motor_name, x)
            elif throttle < 0:
                if throttle < -100:
                    print('Throttle Too HIGH')    
                x = (-8/10)*throttle + 100
                self.set_angle(motor_name, x)
            elif throttle == 0:
                self.set_angle(motor_name, 100)
        else:
            self.get_logger().error("Invalid motor name")
    
    def set_angle(self, motor_name, angle):
        if motor_name in self.motors:
            self.motors[motor_name].angle = angle

    def stop_all_motors(self):
        self.get_logger().info("Stopping all motors")
        for motor_name in self.motors.keys():
            self.set_angle(motor_name, 100)

    def test_motors(self):
        for motor in self.motors:
            self.set_throttle(motor, 15)
            print(motor)
            time.sleep(1)
            self.set_throttle(motor, 0)

    def depth_control(self,current_depth,depth):
        if (depth) > current_depth:
            self.set_throttle('left_vert',(60))
            self.set_throttle('right_vert',(60))
            print("lowering")
        elif (depth) < current_depth:
            self.set_throttle('left_vert',(-60))
            self.set_throttle('right_vert',(-60))
            print("raising")
        else:
            self.set_throttle('left_vert',0)
            self.set_throttle('right_vert',0)
            #print("Not Moving")


def main(args=None):
    rclpy.init(args=args)
    global mode
    global heading
    motor_auv_node = MotorAUV()
    
    try:
        while True:
            mode = input("start or test_motors?: ")
            if mode == 'start':
                break
            elif mode == 'test_motors':
                motor_auv_node.test_motors()
                mode = 'none'
            else:
                print('Try again...')
                mode = 'none'

        print("starting up...")
        
        rclpy.spin(motor_auv_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        motor_auv_node.get_logger().info("Shutting down motorauv node")
    finally:
        motor_auv_node.destroy_node()
        motor_auv_node.stop_all_motors()
        rclpy.shutdown()


depth = 0
speed = 0
heading = 0
timer_count = 0

if __name__ == '__main__':
    main()

