#!/usr/bin/env python
from enum import Enum

import rospy
from visualization_msgs.msg import Marker
from tf.listener import TransformListener
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient, GoalStatus
from perspectives_msgs.srv import StartFact, EndFact
from head_manager.msg import CoordinationSignal, TargetWithExpiration

NODE_NAME = "nathan_expe_supervisor"
NS = "nathan_expe"

MAP_FRAME = "map"
HUMAN_FRAME = "mocap_human-1_footprint"
HUMAN_HEAD_FRAME = "mocap_human-1"

HUMAN_TRIGGER_MARKER_PUB = "{}/human_trigger_marker".format(NS)
COORD_SIGNAL_PUB = "head_manager/head_coordination_signals"
MOVE_BASE_ACTION_SRV = "/move_base"
START_FACT_SRV_NAME = "/uwds_ros_bridge/start_fact"
STOP_FACT_SRV_NAME = "/uwds_ros_bridge/end_fact"

DURATION_BETWEEN_COORD_SIGNAL = 1.9  # ----|##Coordination signal##|<------this duration------>|###Another coordination signal###|-----> time
COORD_SIGNAL_DURATION = 1.9

class ExpeSupervisor:
    class State(Enum):
        IDLE = 0
        RUNNING = 1
        ENDED = 2

    def __init__(self):
        self.marker_pub = rospy.Publisher(HUMAN_TRIGGER_MARKER_PUB, Marker, queue_size=10)
        self.coord_signal = rospy.Publisher(COORD_SIGNAL_PUB, CoordinationSignal, queue_size=1)
        self.tf_listener = TransformListener()
        self.move_base = SimpleActionClient(MOVE_BASE_ACTION_SRV, MoveBaseAction)
        rospy.loginfo("{} waiting for move_base action server.".format(NODE_NAME))
        self.move_base.wait_for_server()
        rospy.loginfo("{} found move_base action server.".format(NODE_NAME))

        self.navigating_fact_id = None
        rospy.loginfo(NODE_NAME + " waiting for underworlds start fact service server.")
        self.start_fact = rospy.ServiceProxy(START_FACT_SRV_NAME, StartFact)
        self.start_fact.wait_for_service()
        rospy.loginfo(NODE_NAME + " found underworlds start fact service server.")
        rospy.loginfo(NODE_NAME + " waiting for underworlds stop fact service server.")
        self.end_fact = rospy.ServiceProxy(STOP_FACT_SRV_NAME, EndFact)
        self.end_fact.wait_for_service()
        rospy.loginfo(NODE_NAME + " found underworlds stop fact service server.")

        self.coordination_human_timer = None
        self.trigger_zone_bb = BoundingBox(3.0, 13.0, 5.0, 16.0)
        self.state = self.State.IDLE
        rospy.Timer(rospy.Duration(1.0), lambda _: self.draw_trigger_zone(False), oneshot=True)
        rospy.Timer(rospy.Duration(0.1), self.loop)

    def draw_trigger_zone(self, activated):
        marker = Marker()
        marker.header.frame_id = MAP_FRAME
        marker.header.stamp = rospy.Time(0)
        marker.lifetime = rospy.Duration(0)
        marker.ns = NS
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (self.trigger_zone_bb[0][0] + self.trigger_zone_bb[1][0]) / 2
        marker.pose.position.y = (self.trigger_zone_bb[0][1] + self.trigger_zone_bb[1][1]) / 2
        marker.pose.position.z = 0.25
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = abs(self.trigger_zone_bb[0][0] - self.trigger_zone_bb[1][0]) / 2
        marker.scale.y = abs(self.trigger_zone_bb[0][1] - self.trigger_zone_bb[1][1]) / 2
        marker.scale.z = 0.5
        marker.color.a = 0.5
        marker.color.r = 0.09 if activated else 0.18
        marker.color.g = 0.15 if activated else 0.26
        marker.color.b = 0.34 if activated else 0.46
        self.marker_pub.publish(marker)

    def send_look_at_human(self):
        coord = CoordinationSignal()
        t = TargetWithExpiration()
        t.target.header.stamp = rospy.Time.now()
        t.target.header.frame_id = HUMAN_HEAD_FRAME
        t.duration = COORD_SIGNAL_DURATION
        t.regex_end_condition = ""
        t.target.point.x = 0.0
        t.target.point.y = 0.0
        t.target.point.z = 0.0

        coord.header.frame_id = HUMAN_HEAD_FRAME
        coord.header.stamp = rospy.Time.now()
        coord.priority = 255
        coord.expiration = rospy.Time.now() + rospy.Duration(3)
        coord.predicate = ""
        coord.regex_end_condition = ""
        coord.targets = [t]
        self.coord_signal.publish()

    def start_navigation(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = MAP_FRAME
        goal.target_pose.pose.position.x = 8.0
        goal.target_pose.pose.position.y = 15.5
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self.start_fact()
        resp = self.start_fact("base", "isNavigating(robot)", rospy.Time.now().to_sec(), False)
        if resp.success:
            self.navigating_fact_id = resp.fact_id

        self.coordination_human_timer = rospy.Timer(
            rospy.Duration(DURATION_BETWEEN_COORD_SIGNAL + COORD_SIGNAL_DURATION), self.send_look_at_human)
        self.move_base.send_goal(goal)

    def stop_navigation(self):
        s = self.move_base.get_state()
        if s != GoalStatus.SUCCEEDED or s != GoalStatus.ABORTED or s != GoalStatus.PREEMPTED:
            self.move_base.cancel_all_goals()

        if self.navigating_fact_id is not None:
            resp = self.end_fact("base", self.move_to_fact_id)
            if resp.success:
                self.move_to_fact_id = None

        if self.coordination_human_timer is not None:
            self.coordination_human_timer.shutdown()

    def loop(self, _):
        t_human, r_human = self.tf_listener.lookupTransform(HUMAN_FRAME, MAP_FRAME, rospy.Time(0))
        if self.state == self.State.IDLE:
            if self.trigger_zone_bb.is_in(t_human[0], t_human[1]):
                self.state = self.State.RUNNING
                self.start_navigation()
                rospy.loginfo("{} idle -> running".format(NODE_NAME))
        elif self.state == self.State.RUNNING:
            s = self.move_base.get_state()
            if s == GoalStatus.SUCCEEDED or s == GoalStatus.ABORTED or s == GoalStatus.PREEMPTED:
                self.stop_navigation()
                rospy.loginfo("{} running -> ended".format(NODE_NAME))
                self.state = self.State.ENDED

    def run(self):
        rospy.loginfo("{} online".format(NODE_NAME))
        rospy.spin()


class BoundingBox:
    def __init__(self, x0, y0, x1, y1):
        self.bb = None
        self.set_bounds(x0, y0, x1, y1)

    def set_bounds(self, x0, y0, x1, y1):
        self.bb = ((min(x0, x1), min(y0, y1)), (max(x0, x1), max(y0, y1)))

    def is_in(self, x, y):
        return self.bb[0][0] <= x <= self.bb[1][0] and self.bb[0][1] <= y <= self.bb[1][1]




if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    ExpeSupervisor().run()


