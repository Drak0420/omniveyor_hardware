#!/usr/bin/env python

# modified from teleop_twist_keyboard package (http://wiki.ros.org/teleop_twist_keyboard)
# author: Austin Hendrix (namniart@gmail.com)
# adaptation made by Haoguang Yang

from __future__ import print_function

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import os

from geometry_msgs.msg import Twist
from std_msgs.msg import Byte

import sys, select, termios, tty

class keyboardTwistTeleop:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.msg = """
        Reading from the keyboard  and Publishing to Twist!
        ---------------------------
        Moving around:
        u    i    o
        j    k    l
        m    ,    .

        For Holonomic mode (strafing), hold down the shift key:
        ---------------------------
        U    I    O
        J    K    L
        M    <    >

        t : up (+z)
        b : down (-z)

        anything else : stop

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        p/P : stop/start the base motion

        CTRL-C to quit
        """

        self.moveBindings = {
                'i':(1,0,0,0),
                'o':(1,0,0,-1),
                'j':(0,0,0,1),
                'l':(0,0,0,-1),
                'u':(1,0,0,1),
                ',':(-1,0,0,0),
                '.':(-1,0,0,1),
                'm':(-1,0,0,-1),
                'O':(1,-1,0,0),
                'I':(1,0,0,0),
                'J':(0,1,0,0),
                'L':(0,-1,0,0),
                'U':(1,1,0,0),
                '<':(-1,0,0,0),
                '>':(-1,-1,0,0),
                'M':(-1,1,0,0),
                't':(0,0,1,0),
                'b':(0,0,-1,0),
            }

        self.speedBindings={
                'q':(1.1,1.1),
                'z':(.9,.9),
                'w':(1.1,1),
                'x':(.9,1),
                'e':(1,1.1),
                'c':(1,.9),
            }
        
        velCmdTopic = rospy.get_param('velocity_command_topic', 'cmd_vel')
        enaCmdTopic = rospy.get_param('control_mode_topic', 'control_mode')
        self.pub = rospy.Publisher(velCmdTopic, Twist, queue_size = 1)
        self.pub2 = rospy.Publisher(enaCmdTopic, Byte, queue_size = 1)
        self.speed = rospy.get_param("~speed", 0.5)
        self.turn = rospy.get_param("~turn", 1.0)
        rospy.on_shutdown(self.shutdown)
        # hold there until the subsecribers are ready
        r = rospy.Rate(30)
        while not self.pub2.get_num_connections():
            if not rospy.is_shutdown():
                r.sleep()
            else:
                return

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        res, _, _ = select.select([sys.stdin], [], [], 1.0)
        key = '' if not res else sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def run(self):
        x = 0
        y = 0
        z = 0
        th = 0
        status = 0
        try:
            if not rospy.is_shutdown():
                print(self.msg)
                print(self.vels(self.speed, self.turn))
                self.pub2.publish(Byte(data=1))
            while not rospy.is_shutdown():
                key = self.getKey() # blocks here
                if key in self.moveBindings.keys():
                    x = self.moveBindings[key][0]
                    y = self.moveBindings[key][1]
                    z = self.moveBindings[key][2]
                    th = self.moveBindings[key][3]
                elif key in self.speedBindings.keys():
                    self.speed *= self.speedBindings[key][0]
                    self.turn *= self.speedBindings[key][1]
                    print(self.vels(self.speed, self.turn))
                    if (status == 14):
                        print(self.msg)
                    status = (status + 1) % 15
                elif not key:
                    pass
                else:
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    if (key == '\x70'):
                        self.pub2.publish(Byte(data=0))
                    elif (key == '\x50'):
                        self.pub2.publish(Byte(data=1))
                    elif (key == '\x03'):
                        break
                twist = Twist()
                twist.linear.x = x*self.speed; twist.linear.y = y*self.speed; twist.linear.z = z*self.speed
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*self.turn
                self.pub.publish(twist)
            self.pub2.publish(Byte(data=0))
        except Exception as e:
            print(e)

    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.pub.publish(twist)
        self.pub2.publish(Byte(data=0))
        

if __name__=="__main__":
    rospy.init_node('teleop_twist_keyboard')
    kb = keyboardTwistTeleop()
    kb.run()
