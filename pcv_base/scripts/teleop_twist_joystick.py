#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Byte
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class joystickTeleop:
    def __init__(self):
        self.msg = """
Reading from the controller and publishing commands!
___________________________________________________
LJoy: Normal operation - Drive forward/back + rotate
RJoy: Strafe
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
"""
        rospy.Subscriber("joy", Joy, self.joy_cb)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.ena_pub = rospy.Publisher("control_mode", Byte, queue_size=1)
        self.speed = rospy.get_param("~speed", 0.5)
        self.turn = rospy.get_param("~turn", 1.0)
        self.ena = False
        self.en_ljoy_hort = True
        self.speed_line_written = 1
        self.time = time.time()
        self.prevOption = 0.0
        self.prevCircle = 0
        self.scaling_factor = {-1.0: 0.9, 1.0: 1.1}
        rospy.on_shutdown(self.shutdown)
        # hold until subscribers ready
        """
        r = rospy.Rate(30.0)
        while not self.ena_pub.get_num_connections():
            if not rospy.is_shutdown():
                r.sleep()
            else:
                print(self.msg)
                self.base_disable()
                return
        """

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
        # High Trigger toggle
        if option != self.prevOption:
            self.prevOption = option
            if option == 1:
                self.ena = not self.ena
                self.ena_pub.publish(Byte(data=1 if self.ena else 0))
        if circle != self.prevCircle:
            self.prevCircle = circle
            if circle == 1:
                self.en_ljoy_hort = not self.en_ljoy_hort
        if dpad_hort or dpad_vert:
            if time.time() - self.time > 0.1:
                self.turn *= self.scaling_factor.get(-dpad_hort, 1)
                self.speed *= self.scaling_factor.get(dpad_vert, 1)
                self.time = time.time()
        if self.ena:  # Motors enabled and deadman engaged
            # Convert motion into Twist() args
            x = ljoy_vert / 2 + rjoy_vert / 2  # divide by 2 to add up to max of 1
            y = rjoy_hort / 2
            omega = ljoy_hort * self.en_ljoy_hort
            if l2_en:
                omega += -(l2 - 1) / 2
            if r2_en:
                omega += (r2 - 1) / 2
            omega = min(max(omega, -1), 1)  # ensure within -1,1
        # send out message
        cmd = Twist()
        cmd.linear.x = self.speed * x
        cmd.linear.y = self.speed * y
        cmd.linear.z = cmd.angular.x = cmd.angular.y = 0
        cmd.angular.z = self.turn * omega
        self.vel_pub.publish(cmd)

        if cross:
            print(self.msg)
        print(self.status())
        # self.debug(msg, cmd)

    def main(self):
        # enable the robot and enters velocity mode
        self.base_enable()
        rospy.spin()
        self.base_disable()

    def debug(self, msg, cmd):
        print("Axes:" + str(msg.axes))
        print("Buttons:" + str(msg.buttons))
        print("Sent" + str(cmd))

    def status(self):
        return f"Motors enabled:{self.ena}\nLeft Joystick Turn Enabled:{self.en_ljoy_hort}\n{self.vels()}"

    def vels(self):
        return f"Vel: speed {self.speed:.4f}\tturn {self.turn:.4f} "

    def base_disable(self):
        self.ena_pub.publish(Byte(0))

    def base_enable(self):
        self.ena_pub.publish(Byte(1))

    def shutdown(self):
        self.base_disable()


if __name__ == "__main__":
    rospy.init_node("teleop_twist_joystick")
    js = joystickTeleop()
    js.main()
