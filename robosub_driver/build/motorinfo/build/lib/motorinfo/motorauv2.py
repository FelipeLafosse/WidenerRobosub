#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import Float64MultiArray, String, Int64MultiArray
import time

#dep = input("Enter depth in meters: ")

class MotorAUV(Node):
    def __init__(self):
        super().__init__('motor_auv_node')
        self.hat = ServoKit(channels=16)

        # Defining motors using ServoKit
        self.back_left = self.hat.servo[6] #Lower Number for forward
        self.left_vert = self.hat.servo[1]
        self.right_vert = self.hat.servo[3]
        self.back_right = self.hat.servo[5]
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

        # Subscription to sensor data
        self.sensor_subscription = self.create_subscription(Float64MultiArray, 'sensor_data', self.sensor_callback, 10)
        
        # Subscription to color topic
        self.color_sub = self.create_subscription(String, 'color_topic', self.camera_callback, 10)
        self.get_logger().info("Motor Control Node initialized and subscribed to sensor data.")

        #subscrioption to command topic
        self.direct = self.create_subscription(Int64MultiArray, 'drive_topic', self.motion_callback, 10)

    def motion_callback(self, msg):
        direction = msg.data[0]
        duration = msg.data[1]
        angle = msg.data[2]
        self.get_logger().info(f"Received direction: {msg.data[0]}, duration: {msg.data[1]}, angle: {msg.data[2]}")

        # Handle the received direction
        if direction == 1:
            self.move_forward(duration)
        elif direction == 2:
            self.move_backward(duration)
        elif direction == 3:
            self.rotate_left(duration)
        elif direction == 4:
            self.rotate_right(duration)
        elif direction == 5:
            self.move_up(duration)
        elif direction == 6:
            self.move_down(duration)
        else:
            self.get_logger().error("Invalid direction received")

    def sensor_callback(self, msg):
        #Unpack sensor data
        depth = msg.data[0]
        acceleration = msg.data[3:6]  # X, Y, Z acceleration
        gyro = msg.data[6:9]         # X, Y, Z angular velocity

        self.get_logger().info(f"Received Sensor Data - Depth: {depth:.2f}, Acceleration: {acceleration}, Gyro: {gyro}")
        #desdepth = float(input("Input desired depth: "))
        #desdepth = 1
        # Stabilization logic based on sensor data
        

    def camera_callback(self, msg):
        color = msg.data
        color_dir = self.get_motor_speed(color)
        self.get_logger().info(f"Received color: {color}, Setting AUV direction to: {color_dir}")

    def get_motor_direction(self, color):
        speed_map = {
            "blue": "forward", 
            "red": "backward", 
            "yellow": "rot_180"
        }
        return speed_map.getcolor(color, 0)
    
    def set_motor_throttle(self, motor_name, throttle):
        if motor_name in self.motors:
            self.motors[motor_name].throttle = throttle
        else:
            self.get_logger().error("Invalid motor name")
    
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

    def stop_motors_after_delay(self):
        self.get_logger().info("Stopping motors after x seconds.")
        self.stop_all_motors()
        # Optionally, stop the timer after it triggers once
        self.timer.cancel()

    def test_gripper(self):
        self.get_logger().info("Testing gripper")
        self.set_gripper_position(0.0)  # Neutral (open)
        self.timer = self.create_timer(2.0, self.stop_motors_after_delay)
        self.set_gripper_position(1.0)  # Close
        self.timer = self.create_timer(2.0, self.stop_motors_after_delay)
        self.set_gripper_position(0.0)  # Neutral (open)
        self.timer = self.create_timer(2.0, self.stop_motors_after_delay)

    def move_forward(self,dur):
        self.get_logger().info("Moving forward")
        self.set_angle("back_left", 50)
        self.set_angle("back_right", 140)
        self.set_angle("front_left", 50)
        self.set_angle("front_right", 140)
        self.timer = self.create_timer(dur, self.stop_motors_after_delay)

    def move_backward(self,dur):
        self.get_logger().info("Moving backward")
        self.set_angle("back_left", 140)
        self.set_angle("back_right", 50)
        self.set_angle("front_left", 140)
        self.set_angle("front_right", 50)
        self.timer = self.create_timer(dur, self.stop_motors_after_delay)

    def move_right(self,dur):
        self.get_logger().info("Moving right")
        self.set_angle("back_left", 85)
        self.set_angle("back_right", 105)
        self.set_angle("front_left", 105)
        self.set_angle("front_right", 85)
        self.timer = self.create_timer(dur, self.stop_motors_after_delay)

    def move_left(self,dur):
        self.get_logger().info("Moving left")
        self.set_angle("back_left", 105)
        self.set_angle("back_right", 85)
        self.set_angle("front_left", 85)
        self.set_angle("front_right", 105)
        self.timer = self.create_timer(dur, self.stop_motors_after_delay)

    def rotate_left(self,dur):
        self.get_logger().info("Rotating left")
        self.set_angle("back_left", 105)
        self.set_angle("back_right", 85)
        self.set_angle("front_left", 105)
        self.set_angle("front_right", 85)
        self.timer = self.create_timer(dur, self.stop_motors_after_delay)

    def rotate_right(self,dur):
        self.get_logger().info("Rotating right")
        self.set_angle("back_left", 85)
        self.set_angle("back_right", 105)
        self.set_angle("front_left", 85)
        self.set_angle("front_right", 105)
        self.timer = self.create_timer(dur, self.stop_motors_after_delay)

    def stop_all_motors(self):
        self.get_logger().info("Stopping all motors")
        for motor_name in self.motors.keys():
            self.set_angle(motor_name, 100)
    
    def stop_vert_motors(self):
        for motor_name in self.vertmotors.keys():
            self.set_angle(motor_name, 100)

def main(args=None):
    rclpy.init(args=args)
    motor_auv_node = MotorAUV()
    # This keeps the node running and automatically processes any incoming messages
    rclpy.spin(motor_auv_node)

    motor_auv_node.get_logger().info("Motor AUV Node running and listening for messages.")
    

    
    # Clean up and shutdown when spinning is done (typically after Ctrl+C)
    motor_auv_node.destroy_node()
    motor_auv_node.stop_all_motors()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

