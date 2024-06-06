#!/usr/bin/env python

import time

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
    Twist,
)
from omniveyor_common.msg import electricalStatus
from sensor_msgs.msg import Joy
from std_msgs.msg import Byte, Empty


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
    CMD_HELP = (
        "CONTROLS\n\r"
        "___________________________________________________\n\r"
        "L1 and R1: One must be held down while manually manuveuring\n\r"
        "LJoy: Normal operation - moves f/b w/ rotation to side\n\r"
        "RJoy: Strafe - moves in direction w/o rotation\n\r"
        "L2: Rotate Left\n\r"
        "R2: Rotate Right\n\r"
        "DPad\n\r"
        "\tUp: Increase global speed by 10%\n\r"
        "\tDown: Decrease global speed by 10%\n\r"
        "\tRight: Increase rotation speed by 10%\n\r"
        "\tLeft: Decrease rotation speed by 10%\n\r"
        "OPTION: Enable/Disable motors\n\r"
        "Left Joystick Press: Enable/Disable rotation with left joystick\n\r"
        "Share: Reset robot to initial position in software\n\r"
        "\t\tRequires manual manuveur to desired location first!\n\r"
        "Cross: Help Menu\n\r"
        "Triangle: Send trigger message to goal sender node\n\r"
        "Square: Cancel current goal\n\r"
    )

    def __init__(self):
        self.speed = float(rospy.get_param("~speed", 0.5))
        self.turn = float(rospy.get_param("~turn", 1.0))
        self.ena = False
        self.en_ljoy_hort = True
        self.speed_line_written = 1
        self.prev_option = 0
        self.prev_ljoy_press = 0
        self.prev_share = 0
        self.prev_triangle = 0
        self.prev_square = 0
        self.prev_status = ""
        self.scaling_factor = {-1.0: 0.9, 1.0: 1.1}

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.ena_pub = rospy.Publisher("control_mode", Byte, queue_size=1)
        self.goal_trig_pub = rospy.Publisher("/goal_send_trigger", Empty, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=1
        )
        self.reset_pose_pub = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, queue_size=1
        )
        rospy.Subscriber("joy", Joy, self.joy_cb)
        rospy.Subscriber("electricalStatus", electricalStatus, self.status_cb)
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
        self.button_press(ljoy_press, "prev_ljoy_press", "en_ljoy_hort")
        self.button_press(share, "prev_share", None, self.reset_pose)
        self.button_press(triangle, "prev_triangle", None, self.send_goal_trigger)
        self.button_press(square, "prev_square", None, self.cancel_goal)
        if dpad_hort or dpad_vert:
            self.update_speed_and_turn(dpad_hort, dpad_vert)

        if self.ena and (l1 or r1):
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
            rospy.loginfo(self.CMD_HELP)

        # Only print status if new status
        status = self.status()
        if status != None:
            rospy.loginfo(status)
            self.prev_stats = status
        # print(self.debug_msg(msg, cmd))

    @debounce(30)
    def status_cb(self, msg):
        # Only run log every x seconds to avoid spam
        info_msg = (
            "Voltages\n\r"
            f"Steering: {round(msg.steer_1_Volt, 3)} {round(msg.steer_2_Volt, 3)}"
            f" {round(msg.steer_3_Volt, 3)} {round(msg.steer_4_Volt, 3)}\n\r"
            f"Roll: {round(msg.roll_1_Volt, 3)} {round(msg.roll_2_Volt, 3)} "
            f"{round(msg.roll_3_Volt, 3)} {round(msg.roll_4_Volt, 3)}\r"
        )
        rospy.loginfo(info_msg)

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
            if curr_value == 1:
                if toggle_attr:
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

    def status(self):
        msg = (
            "Motors enabled: "
            + str(self.ena)
            + "  Left Joystick Turn Enabled: "
            + str(self.en_ljoy_hort)
            + "  Vel: speed {speed:.4f} turn {turn:.4f}\r".format(
                speed=self.speed,
                turn=self.turn,
            )
        )
        if msg != self.prev_status:
            self.prev_status = msg
            return msg

    def reset_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose = PoseWithCovariance(pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)))
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        self.reset_pose_pub.publish(msg)
        rospy.loginfo("Sent pose reset command \r")
        return

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
        rospy.loginfo("Sent goal trigger message\r")
        self.goal_trig_pub.publish(Empty())

    def cancel_goal(self):
        rospy.loginfo("Sent cancel message\r")
        self.goal_cancel_pub.publish(GoalID())

    def shutdown(self):
        self.base_disable()


if __name__ == "__main__":
    try:
        rospy.init_node("teleop_twist_joystick")
        js = joystickTeleop()
        js.main()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Keyboard Interrupt")
