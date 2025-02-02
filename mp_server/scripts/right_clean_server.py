import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool
from mp_server_msgs.msg import PickPlaceAction, PickPlaceActionGoal, PickPlaceActionResult
import actionlib
from ur_msgs.srv import SetIO


class Motion_planner:

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.loginfo("Initializing motion planner")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "right_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.waypoints = []

        # create action server for pick and place
        self.clean_server = actionlib.SimpleActionServer(
            "right_clean", PickPlaceAction, self.clean_callback, auto_start=False
        )

        self.set_io_client = rospy.ServiceProxy("/right/ur_hardware_interface/set_io", SetIO)
        self.set_io_client.wait_for_service()

        self.clean_server.start()



    def clean_callback(self, goal:PickPlaceActionGoal):
        rospy.loginfo("Received clean request")
        
        success = self.clean()
        
        # set goal to success
        result = PickPlaceActionResult()
        result.result = success

        if success:
            self.clean_server.set_succeeded(result)
        else:
            self.clean_server.set_aborted(result)


    def clean(self):
        rospy.loginfo("Cleansing the end effector")

        water_bowl_position = Pose()
        water_bowl_position.position.x = 0.32707529223886334
        water_bowl_position.position.y = 0.10471837269250486
        water_bowl_position.position.z = 0.04357432827126255
        water_bowl_position.orientation.x = -0.5825436479626629
        water_bowl_position.orientation.y = -0.8127928845856666
        water_bowl_position.orientation.z = 0.0029525975807408256
        water_bowl_position.orientation.w = 0.001380997027855892

        # plan a cartesian path to cleanse, precleanse -> cleanse and oscillate
        waypoints = []
        precleanse = Pose()
        precleanse = copy.deepcopy(water_bowl_position)
        precleanse.position.z += 0.1 # move up 10 cm
        waypoints.append(copy.deepcopy(precleanse))
        
        cleanse = copy.deepcopy(precleanse)
        cleanse.position.z -= 0.1 # move down 10 cm
        waypoints.append(copy.deepcopy(cleanse))

        # oscilate the end effector 2 times with an amplitude between precleanse and cleanse
        waypoints.append(copy.deepcopy(precleanse))
        waypoints.append(copy.deepcopy(cleanse))
        waypoints.append(copy.deepcopy(precleanse))



        # plan a cartesian path
        try : 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01  # eef_step
            )
        except Exception as e:
            print(e)
            return False

        # display the plan
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        # execute the plan
        rospy.loginfo("Executing cleanse<->precleanse plan")
        try : 
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
        except Exception as e:
            print(e)
            return False

        self.set_io_client(1,12,1)
        rospy.sleep(0.3)
        self.set_io_client(1,12,0)
        rospy.sleep(0.3)
        self.set_io_client(1,12,1)
        rospy.sleep(0.3)
        self.set_io_client(1,12,0)
        rospy.sleep(0.3)
        self.set_io_client(1,12,1)
        rospy.sleep(0.3)
        self.set_io_client(1,12,0)
        rospy.sleep(0.3)

        return True
        
if __name__  == "__main__":
    rospy.init_node("right_clean_server", anonymous=True)
    mp = Motion_planner()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Done")
    sys.exit(0)
