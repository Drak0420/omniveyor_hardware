#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Byte, Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def debounce(interval):
    def decorator(func):
        last_executed = 0

        def wrapper(*args, **kwargs):
            nonlocal last_executed
            if time.time() - last_executed > interval:
                last_executed = time.time()
                return func(*args, **kwargs)

        return wrapper

    return decorator


class joystickTeleop:
    CMD_HELP = """
                    CONTROLS
___________________________________________________
LJoy: Normal operation - moves f/b w/ rotation to sides
RJoy: Strafe - moves in direction w/o rotation
L2: Rotate Left
R2: Rotate Right
DPad
    Up: Increase global speed by 10%
    Down: Decrease global speed by 10%
    Right: Increase rotation speed by 10%
    Left: Decrease rotation speed by 10%
    
OPTION: Enable/Disable motors
Circle: Enable/Disable rotation with left joystick
Cross: Help Menu
Triangle: Send trigger message to goal publisher
            Requires a node to recieve msg!
"""

    def __init__(self):
        rospy.Subscriber("joy", Joy, self.joy_cb)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.ena_pub = rospy.Publisher("control_mode", Byte, queue_size=1)
        self.goal_trig_pub = rospy.Publisher("/aruco_goal_send", Empty, queue_size=1)
        self.speed = float(rospy.get_param("~speed", 0.5))
        self.turn = float(rospy.get_param("~turn", 1.0))
        self.ena = False
        self.en_ljoy_hort = True
        self.speed_line_written = 1
        self.prev_option = 0.0
        self.prev_circle = 0
        self.prev_triangle = 0
        self.scaling_factor = {-1.0: 0.9, 1.0: 1.1}
        rospy.on_shutdown(self.shutdown)

    def joy_cb(self, msg):
        # map msg to user_readable values
        ljoy_hort, ljoy_vert, l2, rjoy_hort, rjoy_vert, r2, dpad_hort, dpad_vert = (
            msg.axes
        )
        (
            cross,
            circle,
            triangle,
            square,
            l1,
            r1,
            l2_en,
            r2_en,
            share,
            option,
            ps_logo,
            ljoy_press,
            rjoy_press,
        ) = msg.buttons
        x = y = omega = 0

        # set params before movement
        self.button_press(option, "prev_option", "ena", None, self.toggle_base)
        self.button_press(circle, "prev_circle", "en_ljoy_hort")
        self.button_press(triangle, "prev_triangle", None, None, self.send_goal_trigger)
        if dpad_hort or dpad_vert:
            self.update_speed_and_turn(dpad_hort, dpad_vert)

        if self.ena:
            x = ljoy_vert / 2 + rjoy_vert / 2
            y = rjoy_hort / 2  # divide by 2 for consistency
            omega = ljoy_hort * self.en_ljoy_hort

            # XX_en used to prevent depressed non-zero states
            if l2_en:
                omega += -(l2 - 1) / 2
            if r2_en:
                omega += (r2 - 1) / 2
            omega = min(max(omega, -1), 1)
        # send out message
        cmd = Twist()
        cmd.linear.x = self.speed * x
        cmd.linear.y = self.speed * y
        cmd.linear.z = cmd.angular.x = cmd.angular.y = 0
        cmd.angular.z = self.turn * omega
        self.vel_pub.publish(cmd)

        if cross:
            print(self.CMD_HELP)
        # print(self.status())
        # rospy.logdebug(self.debug_msg(msg, cmd))

    def main(self):
        # enable the robot and enters velocity mode
        self.base_enable()
        rospy.spin()
        self.base_disable()

    def button_press(
        self,
        curr_value,
        prev_attr,
        toggle_attr=None,
        hilow_trig_func=None,
        toggle_func=None,
    ):
        """
        Function to get trigger states from buttons

        :param curr_value: Current button state
        :param prev_attr: Previous button attribute
        :param toggle_attr: Attribute to change on button press
        :param hilow_trig_func: Function to run on button press
        :param toggle_func: Function to run on button press and depress
        """
        if curr_value != getattr(self, prev_attr):
            setattr(self, prev_attr, curr_value)
            # Low -> High State change
            if curr_value == 1 and toggle_attr:
                toggle_value = getattr(self, toggle_attr)
                setattr(self, toggle_attr, not toggle_value)
                if hilow_trig_func:
                    hilow_trig_func()
            if toggle_func:
                toggle_func()

    @debounce(0.1)
    def update_speed_and_turn(self, dpad_hort, dpad_vert):
        self.turn *= self.scaling_factor.get(-dpad_hort, 1)
        self.speed *= self.scaling_factor.get(dpad_vert, 1)

    def debug_msg(self, msg, cmd):
        return (
            "Axes:"
            + str(msg.axes)
            + "\n"
            + "Buttons:"
            + str(msg.buttons)
            + "\n"
            + "Sent"
            + str(cmd)
        )

    @debounce(1)
    def status(self):
        return "Motors enabled:{motor_ena} \
        Left Joystick Turn Enabled:{ljoy_ena} \
        Vel: speed {speed:.4f}\tturn {turn:.4f}\n".format(
            motor_ena=self.ena,
            ljoy_ena=self.en_ljoy_hort,
            speed=self.speed,
            turn=self.turn,
        )

    def base_disable(self):
        self.ena_pub.publish(Byte(0))

    def base_enable(self):
        self.ena_pub.publish(Byte(1))

    def toggle_base(self):
        if self.ena:
            self.base_enable()
        else:
            self.base_disable()

    def send_goal_trigger(self):
        self.goal_trig_pub.publish(Empty())

    def shutdown(self):
        self.base_disable()


if __name__ == "__main__":
    try:
        rospy.init_node("teleop_twist_joystick")
        js = joystickTeleop()
        js.main()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Keyboard Interrupt")
