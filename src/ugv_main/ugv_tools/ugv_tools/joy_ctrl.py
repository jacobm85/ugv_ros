#!/usr/bin/env python
# encoding: utf-8

import os
import time
import getpass
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool, Float32MultiArray


class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)
        self.joy_active = False
        self.buzzer_active = False
        self.led_ctrl_index = 0
        self.led_on = False
        self.cancel_time = time.time()
        self.last_led_toggle_time = time.time()
        self.user_name = getpass.getuser()
        self.linear_gear = 1
        self.angular_gear = 1

        self.pub_goal = self.create_publisher(GoalID, "move_base/cancel", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_buzzer = self.create_publisher(Bool, "Buzzer", 1)
        self.pub_joy_state = self.create_publisher(Bool, "JoyState", 10)
        self.pub_led_ctrl = self.create_publisher(Float32MultiArray, "ugv/led_ctrl", 10)

        self.sub_joy = self.create_subscription(Joy, 'joy', self.button_callback, 10)

        self.declare_parameter('xspeed_limit', 0.2) # Sänkt från 0.5 till 0.3
        self.declare_parameter('yspeed_limit', 0.2) # Sänkt från 0.5 till 0.3
        self.declare_parameter('angular_speed_limit', 0.5) # Sänkt från 1.0 till 0.7
        self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
        self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value

        self.prev_axes = None
        self.prev_buttons = None
        self.last_print_time = 0

    def button_callback(self, joy_data):
        logger = get_logger("JoyTeleop")

        axes = list(joy_data.axes)
        buttons = list(joy_data.buttons)

        if axes != self.prev_axes or buttons != self.prev_buttons:
            current_time = time.time()
            if current_time - self.last_print_time > 1:
                logger.info(f"Debug - Axes: {joy_data.axes}")
                logger.info(f"Debug - Buttons: {joy_data.buttons}")
                self.last_print_time = current_time

        self.prev_axes = axes
        self.prev_buttons = buttons

        if not isinstance(joy_data, Joy):
            return

        def button_pressed(index):
            return len(buttons) > index and buttons[index] == 1

        twist = Twist()
        twist.linear.x = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_gear
        twist.angular.z = self.filter_data(joy_data.axes[0]) * self.angular_speed_limit * self.angular_gear

        if twist.linear.x != 0.0:
            logger.info(f"twist.linear.x: {twist.linear.x}")

        if twist.angular.z != 0.0:
            logger.info(f"twist.angular.z: {twist.angular.z}")

        if self.joy_active:
            self.pub_cmd_vel.publish(twist)

        if self.user_name == "root":
            self.user_jetson(joy_data)
        else:
            self.user_pc(joy_data)

    def user_jetson(self, joy_data):
        logger = get_logger("user_jetson")
        buttons = joy_data.buttons

        def button_pressed(index):
            return len(buttons) > index and buttons[index] == 1

        # Select knapp för IO5 turret lampa
        if button_pressed(6):  # Select button
            current_time = time.time()
            if current_time - self.last_led_toggle_time > 0.5:  # Minimum 0.5 sekunder mellan toggling
                logger.info("Select button pressed for turret LED control")
                self.led_on = not self.led_on
                led_ctrl_msg = Float32MultiArray()
                if self.led_on:
                    led_ctrl_msg.data = [float(0 * self.led_ctrl_index), float(255 * self.led_ctrl_index)]
                else:
                    led_ctrl_msg.data = [float(0), float(0)]
                self.pub_led_ctrl.publish(led_ctrl_msg)
                if self.led_ctrl_index >= 6:
                    self.led_ctrl_index = 0
                self.led_ctrl_index += 1
                self.last_led_toggle_time = current_time

        # Select knapp för IO4 chassie lampor
        if button_pressed(7):  # Start button
            current_time = time.time()
            if current_time - self.last_led_toggle_time > 0.5:  # Minimum 0.5 sekunder mellan toggling
                logger.info("Start button pressed for chassie LED control")
                self.led_on = not self.led_on
                led_ctrl_msg = Float32MultiArray()
                if self.led_on:
                    led_ctrl_msg.data = [float(255 * self.led_ctrl_index), float(0 * self.led_ctrl_index)]
                else:
                    led_ctrl_msg.data = [float(0), float(0)]
                self.pub_led_ctrl.publish(led_ctrl_msg)
                if self.led_ctrl_index >= 6:
                    self.led_ctrl_index = 0
                self.led_ctrl_index += 1
                self.last_led_toggle_time = current_time
                
         # Home knapp för IO4 o IO5 lampor
        if button_pressed(8):  # Home button
            current_time = time.time()
            if current_time - self.last_led_toggle_time > 0.5:  # Minimum 0.5 sekunder mellan toggling
                logger.info("Home button pressed for LED control")
                self.led_on = not self.led_on
                led_ctrl_msg = Float32MultiArray()
                if self.led_on:
                    led_ctrl_msg.data = [float(255 * self.led_ctrl_index), float(255 * self.led_ctrl_index)]
                else:
                    led_ctrl_msg.data = [float(0), float(0)]
                self.pub_led_ctrl.publish(led_ctrl_msg)
                if self.led_ctrl_index >= 6:
                    self.led_ctrl_index = 0
                self.led_ctrl_index += 1
                self.last_led_toggle_time = current_time               

        # Hantera vänstra styrspaken för att stänga av och på navigation
        if button_pressed(9):  # left analog stick button
            logger.info("Select button pressed for cancelling navigation")
            self.cancel_nav()

        if button_pressed(10):  # Buzzer right analog stick button
            logger.info("Buzzer button pressed")
            buzzer_ctrl = Bool()
            self.buzzer_active = not self.buzzer_active
            buzzer_ctrl.data = self.buzzer_active
            for i in range(3):
                self.pub_buzzer.publish(buzzer_ctrl)

        if button_pressed(13):  # Linear gear management
            logger.info("Linear gear management button pressed")
            if self.linear_gear == 1.0:
                self.linear_gear = 1.0 / 3
            elif self.linear_gear == 1.0 / 3:
                self.linear_gear = 2.0 / 3
            elif self.linear_gear == 2.0 / 3:
                self.linear_gear = 1

        if button_pressed(14):  # Angular gear management
            logger.info("Angular gear management button pressed")
            if self.angular_gear == 1.0:
                self.angular_gear = 1.0 / 4
            elif self.angular_gear == 1.0 / 4:
                self.angular_gear = 1.0 / 2
            elif self.angular_gear == 1.0 / 2:
                self.angular_gear = 3.0 / 4
            elif self.angular_gear == 3.0 / 4:
                self.angular_gear = 1.0

    def user_pc(self, joy_data):
        logger = get_logger("user_pc")
        buttons = joy_data.buttons

        def button_pressed(index):
            return len(buttons) > index and buttons[index] == 1


        if button_pressed(4):  # 4
            logger.info("Start button pressed for Buzzer control")
            self.buzzer_active = not self.buzzer_active
            self.pub_buzzer.publish(Bool(data=self.buzzer_active))

        if button_pressed(2):  # 2
            logger.info("Select button pressed for linear gear control")
            if self.linear_gear == 1.0:
                self.linear_gear = 1.0 / 3
            elif self.linear_gear == 1.0 / 3:
                self.linear_gear = 2.0 / 3
            elif self.linear_gear == 2.0 / 3:
                self.linear_gear = 1

        if button_pressed(3):  # 3 Angular gear management
            logger.info("Button for angular gear control pressed")
            if self.angular_gear == 1.0:
                self.angular_gear = 1.0 / 4
            elif self.angular_gear == 1.0 / 4:
                self.angular_gear = 1.0 / 2
            elif self.angular_gear == 1.0 / 2:
                self.angular_gear = 3.0 / 4
            elif self.angular_gear == 3.0 / 4:
                self.angular_gear = 1.0

    def filter_data(self, value):
        if abs(value) < 0.2:
            value = 0.0
        return value

    def cancel_nav(self):
        logger = get_logger("cancel_nav")
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            joy_ctrl = Bool()
            self.joy_active = not self.joy_active
            logger.info(f"Joy state active: {self.joy_active}")
            joy_ctrl.data = self.joy_active
            for i in range(3):
                self.pub_joy_state.publish(joy_ctrl)
                self.pub_cmd_vel.publish(Twist())
            self.cancel_time = now_time


def main():
    rclpy.init()
    joy_ctrl = JoyTeleop('joy_ctrl')
    rclpy.spin(joy_ctrl)


if __name__ == "__main__":
    main()
