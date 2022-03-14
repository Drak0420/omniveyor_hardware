#!/usr/bin/env python

import rospy
from std_msgs.msg import Byte
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class joystickTeleop():
    def __init__(self):
        self.v_lim = [0.5, 0.5, 0.5]
        rospy.Subscriber('joy',Joy, self.joy_cb)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.ena_pub = rospy.Publisher('control_mode', Byte, queue_size = 1)
        rospy.on_shutdown(self.base_disable)
        r = rospy.Rate(30.0)
        while not self.ena_pub.get_num_connections():
            if not rospy.is_shutdown():
                r.sleep()
            else:
                break

    def joy_cb(self, msg):
        cmd = Twist()
        self.vel_x = msg.axes[1]*self.v_lim[0]
        self.vel_y = msg.axes[0]*self.v_lim[1]
        self.omega = msg.axes[2]*self.v_lim[2]
        cmd.linear.x = self.vel_x
        cmd.linear.y = self.vel_y
        cmd.angular.z = self.omega
        self.vel_pub.publish(cmd)

    def main(self):
        # enable the robot and enters velocity mode
        self.ena_pub.publish(Byte(1))
        rospy.spin()
        self.ena_pub.publish(Byte(0))

    def base_disable(self):
        self.ena_pub.publish(Byte(0))

    def base_enable(self):
        self.ena_pub.publish(Byte(1))

if __name__=="__main__":
    rospy.init_node('teleop_twist_joystick')
    js = joystickTeleop()
    js.main()

