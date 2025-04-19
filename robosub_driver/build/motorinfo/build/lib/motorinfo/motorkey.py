#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
import time

class MotorAUV(Node):
    def __init__(self):
        super().__init__('motor_auv_node')
        self.hat = ServoKit(channels=16)

        # Defining motors using ServoKit
        self.back_left = self.hat.servo[6] 
        self.left_vert = self.hat.servo[1]
        self.right_vert = self.hat.servo[3] #Lower Number for forward
        self.back_right = self.hat.servo[5]
        self.front_right = self.hat.servo[7] #Lower Number for forward
        self.front_left = self.hat.servo[10] 
        self.gripper = self.hat.servo[8]  # If you want to keep the gripper - Might need to Fix

        self.back_left.set_pulse_width_range(1000,2000)
        self.left_vert.set_pulse_width_range(1000,2000)
        self.right_vert.set_pulse_width_range(1000,2000)
        self.back_right.set_pulse_width_range(1000,2000)
        self.front_left.set_pulse_width_range(1000,2000)
        self.front_right.set_pulse_width_range(1000,2000)

        self.motors = {
            "back_left": self.back_left,
            "left_vert": self.left_vert,
            "right_vert": self.right_vert,
            "back_right": self.back_right,
            "front_right": self.front_right,
            "front_left": self.front_left,
            "gripper": self.gripper,
        }

        self.vertmotors = {
            "left_vert": self.left_vert,
            "right_vert": self.right_vert,
        }

    def set_angle(self, motor_name, angle):
        if motor_name in self.motors:
            self.motors[motor_name].angle = angle

    def test_motor(self, motor_name):
        self.get_logger().info("\033[31mStarting test in 3 seconds...\033[0m")
        time.sleep(3)
        self.get_logger().info(f"\033[36mTesting motor:\033[0m {motor_name}")
        self.set_angle(motor_name, 100)  # Neutral
        time.sleep(2)
        self.set_angle(motor_name, 110)  # Forward
        time.sleep(1)
        self.set_angle(motor_name, 100)  # Stop/Slow down
        time.sleep(2)
        self.set_angle(motor_name, 80)   # Backwards
        time.sleep(1)
        self.set_angle(motor_name, 100)  # Neutral
        time.sleep(2)

    def set_gripper_position(self, position):
        if "gripper" in self.motors:
            self.motors["gripper"].throttle = position
        else:
            self.get_logger().error("\033[31mGripper motor not initialized\033[0m")

    def test_gripper(self):
        self.get_logger().info("\033[36mTesting gripper\033[0m")
        self.set_gripper_position(0.0)  # Neutral (open)
        time.sleep(2)
        self.set_gripper_position(1.0)  # Close
        time.sleep(2)
        self.set_gripper_position(0.0)  # Neutral (open)
        time.sleep(2)

    def stop_all_motors(self):
        self.get_logger().info("\033[31mStopping all motors\033[0m")
        for motor_name in self.motors.keys():
            self.set_angle(motor_name, 100)

    def stop_vert_motors(self):
        self.get_logger().info("\033[31mStopping vertical motors\033[0m")
        for motor_name in self.vertmotors.keys():
            self.set_angle(motor_name, 100)

    def init_motors(self):
        self.get_logger().info("\033[32mInitializing all motors\033[0m")
        for motor_name in self.motors.keys():
            self.set_angle(motor_name, 100)

    def directionCalc(self, direction, throttle): # a1 = BL, b1 = BR, c1 = FL, d1 = FR
        if direction == 'forward':
            a1 = b1 = c1 = d1 = 90 - throttle
        elif direction == 'backward':
            a1 = b1 = c1 = d1 = 100 + throttle
        elif direction == 'left':
            a1 = d1 = 90 - throttle
            b1 = c1 = 100 + throttle
        elif direction == 'right':
            b1 = c1 = 90 - throttle
            a1 = d1 = 100 + throttle
        elif direction == 'rot_left':
            b1 = d1 = 90 - throttle
            a1 = c1 = 100 + throttle
        elif direction == 'rot_right':
            a1 = c1 = 90 - throttle
            b1 = d1 = 100 + throttle
        return a1, b1, c1, d1

    def setDrive(self, backleft, backright, frontleft, frontright):
        self.set_angle("back_left", backleft)
        self.set_angle("back_right", backright)
        self.set_angle("front_left", frontleft)
        self.set_angle("front_right", frontright)

    def motorDrive(self, direction, throttle, duration): #Range [0-80]
        a1, b1, c1, d1 = self.directionCalc(direction, throttle)
        self.setDrive(a1,b1,c1,d1)
        time.sleep(duration)
        self.stop_all_motors()

    def fourmotor(self, a2, b2, c2, d2, duration): # a1 = BL, b1 = BR, c1 = FL, d1 = FR
        self.get_logger().info("\033[31mStarting test in 3 seconds...\033[0m")
        time.sleep(3)
        self.setDrive(a2,b2,c2,d2)
        time.sleep(duration)
        self.stop_all_motors()

    def twomotor(self, power, duration):
        self.get_logger().info("\033[31mStarting test in 3 seconds...\033[0m")
        time.sleep(3)
        a2 = b2 = power
        self.setDrive(a2,b2,100,100)
        time.sleep(duration)
        self.stop_all_motors()

    def motorVert(self, direction):
        self.get_logger().info("\033[36mTesting motor:\033[0m " + direction)
        if direction == 'up':
            for motor_name in self.vertmotors.keys():
                self.set_angle(motor_name, 120)
        elif direction == 'down':
            for motor_name in self.vertmotors.keys():
                self.set_angle(motor_name, 70)
        time.sleep(15)
        self.stop_vert_motors()

    def changesetting(self):
        setting = input("\033[36mSet Driving Mode:\033[33m default, manual, testing, condrive, exit : \033[0m")
        return setting

    def run(self):
        pastDir, pastpow, pasttime = None, None, None
        repeat_last = False
        self.init_motors()
        setting = input("\033[36mSet Driving Mode:\033[33m default, manual, testing, condrive, exit : \033[0m")
        while rclpy.ok():
            if setting == 'default' or setting == 'manual':
                if not repeat_last:
                    movedir = input("\033[36mEnter Movement to test:\033[33m [forward, backward, left, right, rot_left, rot_right, repeat, return] : \033[0m")
                    dirList = ["forward", "backward", "left", "right", "rot_left", "rot_right", "repeat", "return"]
                    if movedir == 'return':
                        setting = self.changesetting()
                        continue
                    elif movedir == 'repeat' and pastDir is not None:
                        movedir, motorpower, drivetime = pastDir, pastpow, pasttime
                        repeat_last = True
                    else:
                        if movedir in dirList:
                            if setting == 'manual':    
                                motorpower = int(input("\033[36mHow strong?\033[33m [0-80] : \033[0m"))
                                drivetime = int(input("\033[36mHow long to run?\033[33m [Seconds] : \033[0m"))
                            else:
                                motorpower, drivetime = 15, 15
                        else:
                            self.get_logger().error("\033[31mInvalid entry\033[0m")
                            continue
                        pastDir, pastpow, pasttime = movedir, motorpower, drivetime
                        repeat_last = False
                self.get_logger().info("\033[31mStarting test in 5 seconds...\033[0m")
                time.sleep(5)
                self.motorDrive(movedir, motorpower, drivetime)
                watch = input("Continue Testing? [\033[32my\033[0m/\033[31mn\033[0m] : ")
                if watch == 'y':
                    repeat_last = False
                    continue
                elif watch == 'repeat':
                    repeat_last = True
                    continue
                elif watch == 'n':
                    setting = self.changesetting()
                    repeat_last = False
                    continue
                else:
                    self.get_logger().error("\033[31mInvalid entry\033[0m")
                    continue
            elif setting == 'testing':
                motor_name = input("\033[36mEnter motor name to Test:\033[33m [back_left, back_right, front_left, front_right, left_vert, right_vert, gripper, up, down, fourdrive, reardrive, stop, return] : \033[0m")
                if motor_name == 'return':
                    setting = self.changesetting()
                    continue
                elif motor_name in self.motors:
                    self.test_motor(motor_name)
                elif motor_name == "gripper":
                    self.test_gripper()
                elif motor_name in ["up", "down"]:
                    self.motorVert(motor_name)
                elif motor_name == "fourdrive":
                    print("\033[31mThis portion doesn't have calculations yet \033[33m[0-90 for forward, 100-180 for reverse\033[0m")
                    a2, b2, c2, d2 = map(int, input ("\033[36mInput Motor Throttles in order \033[33m[backleft, backright, frontleft, frontright]\033[36m w/ spaces between: \033[0m").split())
                    drivetime2 = int(input("\033[36mHow long to run?\033[33m [seconds] : \033[0m"))
                    self.fourmotor(a2,b2,c2,d2,drivetime2)
                elif motor_name == "reardrive":
                    motorpower2 = int(input("\033[36mHow strong?\033[33m [0-90] : \033[0m"))
                    drivetime3 = int(input("\033[36mHow long to run?\033[33m [seconds] : \033[0m"))
                    self.twomotor(motorpower2, drivetime3)
                elif motor_name == "stop":
                    self.stop_all_motors()
                else:
                    self.get_logger().error("\033[31mInvalid entry\033[0m")
                    continue
            elif setting == 'condrive':
                condir = input("\033[36mEnter motor name to Test:\033[33m [wasd, p, return] : \033[0m")
                if condir == "w": #Forwards
                    self.setDrive(70,70,70,70)
                elif condir == "s": #Backwards
                    self.setDrive(120,120,120,120)
                elif condir == "a": #Rotates Left
                    self.setDrive(120,70,120,70)
                elif condir == "d": #Rotates Right
                    self.setDrive(70,120,70,120)
                elif condir == "p": #Stop Bot
                    self.setDrive(100,100,100,100)
                elif condir == 'return':
                    setting = self.changesetting()
                    continue
                else:
                    self.get_logger().error("\033[31mInvalid entry\033[0m")
                    continue
            elif setting == 'exit':
                break
            else:
                self.get_logger().error("\033[31mInvalid entry\033[0m")
                setting = self.changesetting()
                continue
        self.get_logger().info("\033[32mMovement testing completed\033[0m")

def main(args=None):
    rclpy.init(args=args)
    motor_auv_node = MotorAUV()
    motor_auv_node.run()

    try:
        rclpy.spin(motor_auv_node)
    except KeyboardInterrupt:
        motor_auv_node.get_logger().info("\033[31mShutting down motor control node\033[0m")
    finally:
        motor_auv_node.destroy_node()
        motor_auv_node.stop_all_motors()
        rclpy.shutdown()

if __name__ == '__main__':
    main()