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

msg = """
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

moveBindings = {
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

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('teleop_twist_keyboard')
    
    velCmdTopic = rospy.get_param('velocity_command_topic', 'cmd_vel')
    enaCmdTopic = rospy.get_param('control_mode_topic', 'control_mode')
    pub = rospy.Publisher(velCmdTopic, Twist, queue_size = 1)
    pub2 = rospy.Publisher(enaCmdTopic, Byte, queue_size = 1)
    #mode = Byte(1)
    # hold there until the subsecribers are ready
    r = rospy.Rate(30)
    while not pub2.get_num_connections():
        if not rospy.is_shutdown():
            r.sleep()
        else:
            sys.exit(0)
    pub2.publish(Byte(data=1))
    #os.system('rostopic pub --once /mobile_base_controller/control_mode std_msgs/Byte 1')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x70'):
                    pub2.publish(Byte(data=0))
                elif (key == '\x50'):
                    pub2.publish(Byte(data=1))
                elif (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        #mode.data = 0
        pub2.publish(Byte(data=0))
        #os.system('rostopic pub --once /mobile_base_controller/control_mode std_msgs/Byte 0')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

