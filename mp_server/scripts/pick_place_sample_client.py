# write a an action client for the pick and place action server

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from mp_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult, PerceivePickPlaceAction, PerceivePickPlaceGoal, PerceivePickPlaceResult
import actionlib

class MpClass:
    def __init__(self):
        # self.left_pick_place_client = actionlib.SimpleActionClient("left_pick_place", PerceivePickPlaceAction)
        self.right_pick_place_client = actionlib.SimpleActionClient("right_pick_place", PickPlaceAction)
        rospy.loginfo("waiting for server")
        # self.left_pick_place_client.wait_for_server()
        self.right_pick_place_client.wait_for_server()
        rospy.loginfo("server connected")


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()

    source = PoseStamped()
    source.pose.position.x = 0.004309470434174376
    source.pose.position.y = -0.4944449237825132
    source.pose.position.z = -0.03474578495982718
    source.pose.orientation.x = -0.9627200256717543
    source.pose.orientation.y = -0.26884133702641677
    source.pose.orientation.z = -0.005948643313934827
    source.pose.orientation.w = 0.0293104302109141

    destination = PoseStamped()
    destination.pose.position.x= -0.17088440403993688
    destination.pose.position.y= -0.4054786985242974
    destination.pose.position.z= 0.07514833134718167
    destination.pose.orientation.x = -0.9625925612708138
    destination.pose.orientation.y = -0.26923720474112145
    destination.pose.orientation.z = -0.005001943816401621
    destination.pose.orientation.w = 0.03003113596485159

    rospy.loginfo("Sending pick and place goal")
    pick_place_goal = PickPlaceGoal()
    pick_place_goal.source = source
    pick_place_goal.destination = destination

    mp.right_pick_place_client.send_goal(pick_place_goal)
    mp.right_pick_place_client.wait_for_result()
    pick_place_result = mp.right_pick_place_client.get_result()
    print("Pick and place result : ", pick_place_result.result)