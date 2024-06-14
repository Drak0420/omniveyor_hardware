#!/usr/bin/env python

# modified from teleop_twist_keyboard package (http://wiki.ros.org/teleop_twist_keyboard)
# author: Austin Hendrix (namniart@gmail.com)
# adaptation made by Haoguang Yang
# new adapation made by Jason Zheng

from __future__ import print_function

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Twist,
)
from omniveyor_common.msg import electricalStatus
from std_msgs.msg import Byte, Int8
from pcv_base.scripts.teleop_twist_joystick import joystickTeleop
import sys, select, termios, tty


class keyboardTwistTeleop(joystickTeleop):
    CMD_HELP = (
        "Reading from the keyboard and Publishing to Twist!\n\r"
        "---------------------------\n\r"
        "Moving around:\n\r"
        "u    i    o\n\r"
        "j    k    l\n\r"
        "m    ,    .\n\r"
        "\n\r"
        "For Holonomic mode (strafing), hold down the shift key:\n\r"
        "---------------------------\n\r"
        "U    I    O\n\r"
        "J    K    L\n\r"
        "M    <    >\n\r"
        "t : up (+z)\n\r"
        "b : down (-z)\n\r"
        "\n\r"
        "anything else : stop\n\r"
        "\n\r"
        "Speed Settings\n\r"
        "---------------------------\n\r"
        "q/z : increase/decrease max speeds by 10%\n\r"
        "w/x : increase/decrease only linear speed by 10%\n\r"
        "e/c : increase/decrease only angular speed by 10%\n\r"
        "p/P : stop/start the base motion\n\r"
        "-/_ : set current position as origin\n\r"
        "+/= : fully reset motors\n\r"
        "\n\r"
        "Goal Options\n\r"
        "---------------------------\n\r"
        "1   : Send goal #0\n\r"
        "2   : Send goal #1\n\r"
        "3   : Send goal #2\n\r"
        "4   : Send goal #3\n\r"
        "`/~ : Cancel goal\n\r"
        "\n\r"
        "CTRL-C to quit\n\r"
    )

    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.moveBindings = {
            "i": (1, 0, 0, 0),
            "o": (1, 0, 0, -1),
            "j": (0, 0, 0, 1),
            "l": (0, 0, 0, -1),
            "u": (1, 0, 0, 1),
            ",": (-1, 0, 0, 0),
            ".": (-1, 0, 0, 1),
            "m": (-1, 0, 0, -1),
            "O": (1, -1, 0, 0),
            "I": (1, 0, 0, 0),
            "J": (0, 1, 0, 0),
            "L": (0, -1, 0, 0),
            "U": (1, 1, 0, 0),
            "<": (-1, 0, 0, 0),
            ">": (-1, -1, 0, 0),
            "M": (-1, 1, 0, 0),
            "t": (0, 0, 1, 0),
            "b": (0, 0, -1, 0),
        }

        self.speedBindings = {
            "q": (1.1, 1.1),
            "z": (0.9, 0.9),
            "w": (1.1, 1),
            "x": (0.9, 1),
            "e": (1, 1.1),
            "c": (1, 0.9),
        }

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.ena_pub = rospy.Publisher("control_mode", Byte, queue_size=1)
        self.goal_trig_pub = rospy.Publisher("/goal_send_trigger", Int8, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=1
        )
        self.reset_pose_pub = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, queue_size=1
        )
        rospy.Subscriber("electricalStatus", electricalStatus, self.status_cb)
        self.speed = float(rospy.get_param("~speed", 0.5))
        self.turn = float(rospy.get_param("~turn", 1.0))
        rospy.on_shutdown(self.shutdown)
        # hold there until the subsecribers are ready
        r = rospy.Rate(30)
        try:
            while not self.ena_pub.get_num_connections():
                if not rospy.is_shutdown():
                    r.sleep()
                else:
                    return
        except rospy.exceptions.ROSInterruptException:
            return

    def getKey(self) -> str:
        tty.setraw(sys.stdin.fileno())
        res, _, _ = select.select([sys.stdin], [], [], 1.0)
        key = "" if not res else sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed: float, turn: float) -> str:
        return "currently:\tspeed %s\tturn %s\r" % (speed, turn)

    def main(self):
        x = 0
        y = 0
        z = 0
        th = 0
        status = 0
        try:
            if not rospy.is_shutdown():
                print(self.CMD_HELP)
                print(self.vels(self.speed, self.turn))
                self.base_disable()
            while not rospy.is_shutdown():
                key = self.getKey()  # blocks here
                if key in self.moveBindings.keys():
                    x = self.moveBindings[key][0]
                    y = self.moveBindings[key][1]
                    z = self.moveBindings[key][2]
                    th = self.moveBindings[key][3]
                elif key in self.speedBindings.keys():
                    self.speed *= self.speedBindings[key][0]
                    self.turn *= self.speedBindings[key][1]
                    print(self.vels(self.speed, self.turn))
                    if status == 14:
                        print(self.CMD_HELP)
                    status = (status + 1) % 15
                elif key == "-" or key == "_":
                    self.reset_pose()
                elif key == "+" or key == "=":
                    self.reset_motors()
                elif key == "1":
                    self.send_goal_trigger(0)
                elif key == "2":
                    self.send_goal_trigger(1)
                elif key == "3":
                    self.send_goal_trigger(2)
                elif key == "4":
                    self.send_goal_trigger(3)
                elif key == "`" or key == "~":
                    self.cancel_goal()
                elif not key:
                    pass
                else:
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    if key == "\x70":
                        self.base_disable()
                    elif key == "\x50":
                        self.base_enable()
                    elif key == "\x03":
                        break
                twist = Twist()
                twist.linear.x = x * self.speed
                twist.linear.y = y * self.speed
                twist.linear.z = z * self.speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = th * self.turn
                self.vel_pub.publish(twist)
            self.base_disable()
        except Exception as e:
            print(e)

    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.vel_pub.publish(twist)
        self.base_disable()

    def joy_cb(self, msg):
        raise NotImplementedError("Joy_cb not implemented")

    def debug_msg(self, msg, cmd):
        raise NotImplementedError("debug_msg not implemented")

    def status(self):
        raise NotImplementedError("status not implemented")

    def toggle_base(self):
        raise NotImplementedError("toggle_base not implemented")


if __name__ == "__main__":
    rospy.init_node("teleop_twist_keyboard")
    kb = keyboardTwistTeleop()
    kb.main()
