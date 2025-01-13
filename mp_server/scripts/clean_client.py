# write a an action client for the pick and place action server

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from mp_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
from mp_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib

class MpClass:
    def __init__(self):
        self.left_clean_client = actionlib.SimpleActionClient("left_clean", PickPlaceAction)
        # self.left_clean_client.wait_for_server()
        self.right_clean_client = actionlib.SimpleActionClient("right_clean", PickPlaceAction)
        # self.right_clean_client.wait_for_server()


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()
    dummy = PoseStamped()
    clean_goal = PickPlaceGoal()
    clean_goal.source = dummy
    clean_goal.destination = dummy
    rospy.loginfo("Sending clean goal")
    # mp.left_clean_client.send_goal(clean_goal)
    # mp.left_clean_client.wait_for_result()
    # clean_result = mp.left_clean_client.get_result()
    # print("clean result : ", clean_result.result)
    # print("Done")
    mp.right_clean_client.send_goal(clean_goal)
    mp.right_clean_client.wait_for_result()
    clean_result = mp.right_clean_client.get_result()
    print("clean result : ", clean_result.result)
    print("Done")