import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

MSG_HEADER = '[Way Points Manager] '


class WayPointsManager:

    def __init__(self):
        rospy.init_node('way_points_manager')
        rospy.loginfo(MSG_HEADER + 'initialisation started')
        self.load_waypoints()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo(MSG_HEADER + 'wait 10s for server')
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr(MSG_HEADER + 'move base is not ready.')
            rospy.signal_shutdown('no move_base')
            exit()
        rospy.loginfo(MSG_HEADER + 'connected to move base')
        rospy.loginfo(MSG_HEADER + 'initialisation completed')

    def load_waypoints(self):
        rospy.loginfo(MSG_HEADER + 'start to load waypoints')
        waypoints_raw = rospy.get_param('way_points_manager/waypoints')
        self.waypoints = list()
        self.current_sequence = 0
        for waypoint_raw in waypoints_raw:
            # print(waypoint_raw)
            self.waypoints.append(Pose(Point(*waypoint_raw['pose']['point']), Quaternion(*waypoint_raw['pose']['orientation'])))
        rospy.loginfo(MSG_HEADER + str(len(self.waypoints)) + ' waypoints loaded')

    def active_cb(self):
        rospy.loginfo(MSG_HEADER + 'Goal pose ' + str(self.current_sequence + 1) + ' is acepted')

    def feedback_cb(self, feedback):
        rospy.loginfo(MSG_HEADER + 'waypoint ' + str(self.current_sequence + 1) + 'in progress')

    def send_waypoint(self):
        next_goal = MoveBaseGoal()
        next_goal.target_pose.header.frame_id = 'map'
        next_goal.target_pose.header.stamp = rospy.Time.now()
        next_goal.target_pose.pose = self.waypoints[self.current_sequence]

        rospy.loginfo(str(self.waypoints[self.current_sequence]))
        self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.loginfo(MSG_HEADER + 'waypoint ' + str(self.current_sequence + 1) + ' has been sent')

    def goal_reached(self):
        rospy.loginfo(MSG_HEADER + 'all waypoints have been completed')
        rospy.signal_shutdown('completed')
        exit()

    def waypoint_aborted(self):
        rospy.loginfo(MSG_HEADER + 'waypoint ' + str(self.current_sequence) + ' has been aborted')
        rospy.signal_shutdown('aborted')
        exit()

    def waypoint_rejected(self):
        rospy.loginfo(MSG_HEADER + 'waypoint ' + str(self.current_sequence) + 'has been rejected')
        rospy.signal_shutdown('rejected')
        exit()

    def done_cb(self, status, result):
        self.current_sequence += 1
        if status == 2 or status == 8:
            rospy.loginfo(MSG_HEADER + 'way point ' + str(self.current_sequence) + ' was cancelled')

        if status == 3:
            rospy.loginfo(MSG_HEADER + 'way point ' + str(self.current_sequence) + ' completed')
            if self.current_sequence < len(self.waypoints):
                self.send_waypoint()
            else:
                self.goal_reached()

        if status == 4:
            self.waypoint_aborted()

        if status == 5:
            self.waypoint_rejected()

    def spin(self):
        rospy.loginfo(MSG_HEADER + 'start')
        self.send_waypoint()
        rospy.spin()

