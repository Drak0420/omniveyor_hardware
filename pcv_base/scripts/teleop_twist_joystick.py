#!/usr/bin/env python3

import time
import subprocess

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
from std_msgs.msg import Byte, Int8


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
        "\nSettings\n\r"
        "___________________________________________________\n\r"
        "OPTION: Enable/Disable motors\n\r"
        "Left Joystick Press: Enable/Disable rotation with left joystick\n\r"
        "SHARE: Set current position as origin in software\n\r"
        "\t\tRequires manual manuveur to desired location first!\n\r"
        "PS Logo\n\r"
        "\tIf L1 or R1 pressed, full resets motors\n\r"
        "Otherwise, brings up this help menu\n\r"
        "\nGoal Options\n\r"
        "___________________________________________________\n\r"
        "Triangle: Send goal trigger #0\n\r"
        "Circle: Send goal trigger #1\n\r"
        "Cross: Send goal trigger #2\n\r"
        "Square: Cancel current goal\n\r"
    )

    def __init__(self):
        self.speed = float(rospy.get_param("~speed", 0.5))
        self.turn = float(rospy.get_param("~turn", 1.0))
        self.ena = False
        self.en_ljoy_hort = True
        self.scaling_factor = {-1.0: 0.9, 1.0: 1.1}
        self.prev_states = {}

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.ena_pub = rospy.Publisher("control_mode", Byte, queue_size=1)
        self.goal_trig_pub = rospy.Publisher("/goal_send_trigger", Int8, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=1
        )
        self.reset_pose_pub = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, queue_size=1
        )
        rospy.Subscriber("joy", Joy, self.joy_cb)
        rospy.Subscriber("electricalStatus", electricalStatus, self.status_cb)
        rospy.on_shutdown(self.shutdown)

    def joy_cb(self, msg: Joy):
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

        self.button_press(option, "option", self.toggle_base)
        self.button_press(ljoy_press, "ljoy_press", self.toggle_value, "en_ljoy_hort")
        self.button_press(share, "share", self.reset_pose)
        self.button_press(triangle, "triangle", self.send_goal_trigger, 0)
        self.button_press(circle, "circle", self.send_goal_trigger, 1)
        self.button_press(cross, "cross", self.send_goal_trigger, 2)
        self.button_press(square, "square", self.cancel_goal)
        if l1 or r1:
            self.button_press(ps_logo, "ps_logo", self.reset_motors)
        else:
            self.button_press(ps_logo, "ps_logo", rospy.loginfo, self.CMD_HELP)

        if dpad_hort or dpad_vert:
            self.update_speed_and_turn(dpad_hort, dpad_vert)

        # Move if enabled movement & deadman engaged
        if self.ena and (l1 or r1):
            x = ljoy_vert / 2 + rjoy_vert / 2
            y = rjoy_hort / 2  # divide by 2 for consistency
            omega = ljoy_hort * self.en_ljoy_hort

            # XX_en used to prevent non-zero states when not actuated
            if l2_en:
                omega += -(l2 - 1) / 2
            if r2_en:
                omega += (r2 - 1) / 2
            omega = min(max(omega, -1), 1)
        # send out message
        cmd = Twist()
        cmd.linear.x = self.speed * x
        cmd.linear.y = self.speed * y
        cmd.angular.z = self.turn * omega
        self.vel_pub.publish(cmd)

        # Only print status if new status
        status = self.status()
        if status != None:
            rospy.loginfo(status)
            self.prev_stats = status
        # print(self.debug_msg(msg, cmd))

    @debounce(30)
    def status_cb(self, msg: electricalStatus):
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
        self.base_disable()
        rospy.spin()
        self.base_disable()

    def button_press(self, curr_value, prev_attr: str, func, *args):
        """
        Function to get trigger states from buttons

        :param curr_value: Current button state
        :param prev_attr: Previous button attribute
        :param toggle_attr: Attribute to change on button press
        :param hilow_trig_func: Function to run on button press
        :param toggle_func: Function to run on button press and depress
        """
        prev_state = self.prev_states.get(prev_attr)
        if curr_value != prev_state:
            self.prev_states[prev_attr] = curr_value
            if curr_value == 1:
                func(*args)

    def toggle_value(self, toggle_attr: str):
        toggle_value = getattr(self, toggle_attr)
        setattr(self, toggle_attr, not toggle_value)

    @debounce(0.1)
    def update_speed_and_turn(self, dpad_hort: float, dpad_vert: float):
        self.turn *= self.scaling_factor.get(-dpad_hort, 1)
        self.speed *= self.scaling_factor.get(dpad_vert, 1)

    def debug_msg(self, msg: Joy, cmd: Twist):
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
        prev_status = self.prev_states.get("status")
        if msg != prev_status:
            self.prev_states["status"] = msg
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
        self.ena = not self.ena
        if self.ena:
            self.base_enable()
        else:
            self.base_disable()

    def send_goal_trigger(self, goal_num):
        rospy.loginfo("Sent goal trigger message\r")
        self.goal_trig_pub.publish(Int8(goal_num))

    def cancel_goal(self):
        rospy.loginfo("Sent cancel message\r")
        self.goal_cancel_pub.publish(GoalID())

    def shutdown(self):
        self.base_disable()

    def reset_motors(self):
        subprocess.run(["rosnode", "kill", "pcv_base_node"])


if __name__ == "__main__":
    try:
        rospy.init_node("teleop_twist_joystick")
        js = joystickTeleop()
        js.main()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Keyboard Interrupt")
