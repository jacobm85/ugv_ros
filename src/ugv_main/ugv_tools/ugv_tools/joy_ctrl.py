#!/usr/bin/env python3
# encoding: utf-8

import os
import time
import getpass
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray


class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)

        # Internal states
        self.joy_active = True     # Robot körs direkt
        self.buzzer_active = False
        self.led_on_io4 = False
        self.led_on_io5 = False

        self.cancel_time = time.time()
        self.last_led_toggle_time = time.time()
        self.user_name = getpass.getuser()

        # Speed & gear
        self.linear_gear = 1
        self.angular_gear = 1

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_buzzer = self.create_publisher(Bool, "Buzzer", 1)
        self.pub_joy_state = self.create_publisher(Bool, "JoyState", 10)
        self.pub_led_ctrl = self.create_publisher(Float32MultiArray, "ugv/led_ctrl", 10)
        self.pub_pan_tilt = self.create_publisher(Float32MultiArray, "ugv/pan_tilt_cmd", 10)

        # Subscription
        self.sub_joy = self.create_subscription(Joy, 'joy', self.button_callback, 10)

        # Declare parameters
        self.declare_parameter('xspeed_limit', 0.2)
        self.declare_parameter('yspeed_limit', 0.2)
        self.declare_parameter('angular_speed_limit', 0.5)

        self.xspeed_limit = self.get_parameter('xspeed_limit').value
        self.yspeed_limit = self.get_parameter('yspeed_limit').value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').value

        # Debug state
        self.prev_axes = None
        self.prev_buttons = None
        self.last_print_time = 0

        # SWECOGPT: BEGIN state and params for turret hold-position
        self.pt_pan_deg = 0.0
        self.pt_tilt_deg = 0.0
        self._pt_last_time = time.time()

        # Parameters (override via ROS2 params if wanted)
        self.declare_parameter('pt_pan_speed_deg_s', 120.0)   # speed at full stick
        self.declare_parameter('pt_tilt_speed_deg_s', 120.0)
        self.declare_parameter('pt_pan_min_deg', -180.0)
        self.declare_parameter('pt_pan_max_deg', 180.0)
        self.declare_parameter('pt_tilt_min_deg', -90.0)
        self.declare_parameter('pt_tilt_max_deg', 90.0)
        self.declare_parameter('pt_deadzone', 0.10)

        self.pt_pan_speed = float(self.get_parameter('pt_pan_speed_deg_s').value)
        self.pt_tilt_speed = float(self.get_parameter('pt_tilt_speed_deg_s').value)
        self.pt_pan_min = float(self.get_parameter('pt_pan_min_deg').value)
        self.pt_pan_max = float(self.get_parameter('pt_pan_max_deg').value)
        self.pt_tilt_min = float(self.get_parameter('pt_tilt_min_deg').value)
        self.pt_tilt_max = float(self.get_parameter('pt_tilt_max_deg').value)
        self.pt_deadzone = float(self.get_parameter('pt_deadzone').value)
        # SWECOGPT: END state and params for turret hold-position
        
        # SWECOGPT: BEGIN optional center offsets (default 0 deg)
        self.declare_parameter('pt_pan_center_deg', 0.0)
        self.declare_parameter('pt_tilt_center_deg', 0.0)
        self.pt_pan_center = float(self.get_parameter('pt_pan_center_deg').value)
        self.pt_tilt_center = float(self.get_parameter('pt_tilt_center_deg').value)
        # SWECOGPT: END optional center offsets

    # ----------------------------------------------------
    # MAIN JOYSTICK CALLBACK
    # ----------------------------------------------------
    def button_callback(self, joy_data):
        logger = get_logger("JoyTeleop")
        axes = joy_data.axes
        buttons = joy_data.buttons

        # Debug print when inputs change
        if axes != self.prev_axes or buttons != self.prev_buttons:
            now = time.time()
            if now - self.last_print_time > 1:
                logger.info(f"Axes: {axes}")
                logger.info(f"Buttons: {buttons}")
                self.last_print_time = now

        self.prev_axes = list(axes)
        self.prev_buttons = list(buttons)

        def pressed(idx):
            return len(buttons) > idx and buttons[idx] == 1

        # ----------------------------------------------------
        # ROBOT BASE CONTROL  (LEFT STICK + D-PAD)
        # ----------------------------------------------------
        left_x = self.filter_data(axes[0])   # left stick horizontal
        left_y = self.filter_data(axes[1])   # left stick vertical

        dpad_x = axes[6] if len(axes) > 6 else 0.0
        dpad_y = axes[7] if len(axes) > 7 else 0.0

        # Merge: left stick has priority
        move_x = left_y if abs(left_y) > 0.1 else dpad_y
        move_z = left_x if abs(left_x) > 0.1 else dpad_x

        twist = Twist()
        twist.linear.x = move_x * self.xspeed_limit * self.linear_gear
        twist.angular.z = move_z * self.angular_speed_limit * self.angular_gear

        if self.joy_active:
            self.pub_cmd_vel.publish(twist)

        # ----------------------------------------------------
        # PAN-TILT TURRET CONTROL  (RIGHT STICK)
        # ----------------------------------------------------
        if len(axes) >= 5:
            right_x = float(axes[3])  # SWECOGPT: use raw right stick to avoid double-deadzone
            right_y = float(axes[4])

            # SWECOGPT: BEGIN integrate to hold position, only right stick affects turret
            now = time.time()
            dt = max(1e-3, min(0.2, now - self._pt_last_time))
            self._pt_last_time = now

            def dz(v, d):
                return 0.0 if abs(v) < d else v

            rx = dz(right_x, self.pt_deadzone)
            ry = dz(right_y, self.pt_deadzone)

            # Integrate angles (deg)
            self.pt_pan_deg += rx * self.pt_pan_speed * dt
            self.pt_tilt_deg += ry * self.pt_tilt_speed * dt

            # Clamp limits
            self.pt_pan_deg = max(self.pt_pan_min, min(self.pt_pan_max, self.pt_pan_deg))
            self.pt_tilt_deg = max(self.pt_tilt_min, min(self.pt_tilt_max, self.pt_tilt_deg))

            # Publish normalized to match ugv_driver (driver multiplies by 180)
            pan_tilt = Float32MultiArray()
            pan_tilt.data = [self.pt_pan_deg / 180.0, self.pt_tilt_deg / 180.0]
            self.pub_pan_tilt.publish(pan_tilt)
            # SWECOGPT: END integrate to hold position

        # ----------------------------------------------------
        # CENTER TURRET (Y button)
        # ----------------------------------------------------
        if pressed(10):  # Y button index 3
            self.center_turret()

        # ----------------------------------------------------
        # LIGHTS
        # ----------------------------------------------------
        # X ? IO4 (chassi LED)
        if pressed(2):   # X button (index 2 for Xbox controller)
            self.toggle_io4()

        # A ? IO5 (turret LED)
        if pressed(0):   # A button (index 0)
            self.toggle_io5()

        # ----------------------------------------------------
        # NAV CANCEL (press left stick button)
        # ----------------------------------------------------
        if pressed(9):
            self.cancel_nav()

        # ----------------------------------------------------
        # Buzzer (right stick button)
        # ----------------------------------------------------
        if pressed(10):
            self.buzzer_toggle()

    # ----------------------------------------------------
    # HELPERS
    # ----------------------------------------------------
    def toggle_io4(self):
        now = time.time()
        if now - self.last_led_toggle_time < 0.3:
            return
        self.last_led_toggle_time = now

        self.led_on_io4 = not self.led_on_io4
        msg = Float32MultiArray()
        msg.data = [255 if self.led_on_io4 else 0, 0]
        self.pub_led_ctrl.publish(msg)

    def toggle_io5(self):
        now = time.time()
        if now - self.last_led_toggle_time < 0.3:
            return
        self.last_led_toggle_time = now

        self.led_on_io5 = not self.led_on_io5
        msg = Float32MultiArray()
        msg.data = [0, 255 if self.led_on_io5 else 0]
        self.pub_led_ctrl.publish(msg)

    def filter_data(self, val):
        return 0.0 if abs(val) < 0.5 else val

    def buzzer_toggle(self):
        self.buzzer_active = not self.buzzer_active
        msg = Bool()
        msg.data = self.buzzer_active
        self.pub_buzzer.publish(msg)

    def cancel_nav(self):
        now = time.time()
        if now - self.cancel_time < 0.5:
            return
        self.cancel_time = now

        self.joy_active = not self.joy_active
        msg = Bool()
        msg.data = self.joy_active
        self.pub_joy_state.publish(msg)

        # Stop robot immediately when toggled off
        if not self.joy_active:
            self.pub_cmd_vel.publish(Twist())

    # SWECOGPT: BEGIN helper to center turret
    def center_turret(self):
        # Sätt interna absoluta vinklar till center
        self.pt_pan_deg = self.pt_pan_center
        self.pt_tilt_deg = self.pt_tilt_center
    
        # Publicera direkt (normaliserat [-1..1])
        msg = Float32MultiArray()
        msg.data = [self.pt_pan_deg / 180.0, self.pt_tilt_deg / 180.0]
        self.pub_pan_tilt.publish(msg)
    # SWECOGPT: END helper to center turret


def main():
    rclpy.init()
    node = JoyTeleop("joy_ctrl")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
