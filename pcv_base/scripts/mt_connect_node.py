#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from omniveyor_common.msg import electricalStatus
from mt_connect.mtconnect_adapter import Adapter
from mt_connect.data_item import Sample, Event


class mt_connect_node:
    def __init__(self) -> None:
        self.host = rospy.get_param("~host", "localhost")
        self.port = rospy.get_param("~port", 7878)
        self.adapter = Adapter((self.host, self.port))

        # Data Items
        rospy.Subscriber("electrical_status", electricalStatus, self.elec_status_cb)
        rospy.Subscriber("robot_status", GoalStatus, self.goal_status_cb)
        rospy.Subscriber("map_pose", PoseWithCovarianceStamped, self.position_cb)

        self.elec_status = Sample("elec_status")
        self.adapter.add_data_item(self.elec_status)

        self.execution_status = Event("execution_status")
        self.adapter.add_data_item(self.execution_status)

        self.Xabs = Sample("Xabs")
        self.adapter.add_data_item(self.Xabs)
        self.Yabs = Sample("Yabs")
        self.adapter.add_data_item(self.Yabs)
        self.Aabs = Sample("Aabs")
        self.adapter.add_data_item(self.Aabs)

        self.avail = Event("avail")
        self.adapter.add_data_item(self.avail)

        self.power = Event("power")
        self.adapter.add_data_item(self.power)

        self.adapter.start()
        self.adapter.begin_gather()
        self.avail.set_value("AVAILABLE")
        self.adapter.complete_gather()

        rospy.on_shutdown(self.shutdown)

    def elec_status_cb(self, msg: electricalStatus):
        avg_voltage = (
            msg.steer_1_Volt
            + msg.steer_2_Volt
            + msg.steer_3_Volt
            + msg.steer_4_Volt
            + msg.roll_1_Volt
            + msg.roll_2_Volt
            + msg.roll_3_Volt
            + msg.roll_4_Volt
        ) / 8
        avg_voltage = round(avg_voltage, 3)
        self.adapter.begin_gather()
        self.elec_status.set_value(avg_voltage)
        self.adapter.complete_gather()

    def goal_status_cb(self, msg: GoalStatus):
        self.adapter.begin_gather()
        if msg.status in (
            GoalStatus.PENDING,
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        ):
            self.execution_status.set_value("READY")
        elif msg.status in (
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING,
        ):
            self.execution_status.set_value("ACTIVE")
        self.adapter.complete_gather()

    def position_cb(self, msg: PoseWithCovarianceStamped):
        self.adapter.begin_gather()
        self.Xabs.set_value(msg.pose.pose.position.x / 1000)
        self.Yabs.set_value(msg.pose.pose.position.y / 1000)
        self.Aabs.set_value(msg.pose.pose.orientation.z)
        self.adapter.complete_gather()

    def shutdown(self):
        self.adapter.begin_gather()
        self.power.set_value("OFF")
        self.avail.set_value("UNAVAILABLE")
        self.execution_status.set_value("STOPPED")
        self.adapter.complete_gather()

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("mt_connect_node")
    mt = mt_connect_node()
    mt.main()
