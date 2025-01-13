# write a an action client for the pick and place action server

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from mp_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult, PerceivePickPlaceAction, PerceivePickPlaceGoal, PerceivePickPlaceResult
import actionlib

class MpClass:
    def __init__(self):
        self.left_pick_place_client = actionlib.SimpleActionClient("left_pick_place", PerceivePickPlaceAction)
        # self.right_pick_place_client = actionlib.SimpleActionClient("right_pick_place", PickPlaceAction)
        rospy.loginfo("waiting for server")
        self.left_pick_place_client.wait_for_server()
        # self.right_pick_place_client.wait_for_server()
        rospy.loginfo("server connected")


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()

    source = PoseStamped()
    source.pose.position.x= -0.16855652402671312
    source.pose.position.y= 0.4269696256103897
    source.pose.position.z= 0.17953726983166396
    source.pose.orientation.x = 0.9653943031274262
    source.pose.orientation.y = -0.2607581565937913
    source.pose.orientation.z = -0.004191010953791741
    source.pose.orientation.w = 0.0012077607809772351

    destination = PoseStamped()
    destination.pose.position.x = 0.48169969662069597
    destination.pose.position.y = 0.10677438559918866
    destination.pose.position.z = 0.1828057741661568
    destination.pose.orientation.x = 0.9655861251973878
    destination.pose.orientation.y = -0.26005584641028756
    destination.pose.orientation.z = -0.0037183083958979746
    destination.pose.orientation.w = 0.0007521680640413613
    
    rospy.loginfo("Sending pick and place goal")
    pick_place_goal = PerceivePickPlaceGoal()
    pick_place_goal.prompt.data = "round pineapple slices"
    pick_place_goal.source = source
    pick_place_goal.destination = destination

    mp.left_pick_place_client.send_goal(pick_place_goal)
    mp.left_pick_place_client.wait_for_result()
    pick_place_result = mp.left_pick_place_client.get_result()
    print("Pick and place result : ", pick_place_result.result)