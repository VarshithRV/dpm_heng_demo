import rospy
from open_set_object_detection_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import cv_bridge
import cv2
import sensor_msgs.msg
from PIL import Image, ImageDraw
import numpy as np
import random
import io
import base64
import json
import requests
from geometry_msgs.msg import PointStamped, PoseStamped
from mp_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult, PerceivePickPlaceAction, PerceivePickPlaceResult, PerceivePickPlaceGoal
from mp_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib
import tf2_ros
import tf2_geometry_msgs
import image_geometry
import  threading
import io
import math
import sys


class CentralClient:
    def __init__(self) -> None:

        self.depth_image = None # contains the depth  cv images
        self.camera_info = None # contains the camera info
        self.color_image = None # contains the color cv image
        self.camera_model = image_geometry.PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()

        # transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        # action client for pick and place
        self.left_pick_place_client = actionlib.SimpleActionClient("left_pick_place", PickPlaceAction)
        self.right_pick_place_client = actionlib.SimpleActionClient("right_pick_place", PickPlaceAction)
        
        # action client for moving to preaction position
        self.right_move_preaction_client = actionlib.SimpleActionClient("left_move_preaction", MovePreactionAction)
        self.left_move_preaction_client = actionlib.SimpleActionClient("right_move_preaction", MovePreactionAction)
        
        # action client for cleaning
        self.right_clean_client = actionlib.SimpleActionClient("right_clean", PickPlaceAction)
        
        rospy.loginfo("Waiting for servers now")
        self.right_clean_client.wait_for_server()
        self.right_pick_place_client.wait_for_server()
        self.right_move_preaction_client.wait_for_server()
        self.left_pick_place_client.wait_for_server()
        self.left_move_preaction_client.wait_for_server()

        # Subscriber for azure depth
        self.depth_image_sub = rospy.Subscriber(
            "/depth_to_rgb/image_raw", 
            sensor_msgs.msg.Image, 
            self.depth_image_callback
        )

        self.camera_info_sub = rospy.Subscriber(
            "/depth_to_rgb/camera_info", 
            sensor_msgs.msg.CameraInfo, 
            self.camera_info_callback
        )

        self.color_image_sub = rospy.Subscriber(
            "/rgb/image_raw",
            sensor_msgs.msg.Image,
            self.color_image_callback
        )

        rospy.sleep(0.01)
        rospy.loginfo("All servers are connected, client executing")


    def convert_image_to_text(self,image: Image) -> str:
    # This is also how OpenAI encodes images: https://platform.openai.com/docs/guides/vision
        with io.BytesIO() as output:
            image.save(output, format="PNG")
            data = output.getvalue()
        return base64.b64encode(data).decode("utf-8")
    
    def get_plan(
        self,
        url: str = "https://robot-api-2.glitch.me/handle_request",
        instruction: str = "Put the objects in the bowl in order of most sweet to least sweet.",
        job_type: str = "grounding_and_planning",
    ):
        
        if self.color_image is None:
            rospy.logerr("Color image not received yet")
            return
        
        image = self.color_image
        # image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        data = {
            "instruction": instruction,
            "images": [dict(base64_string=self.convert_image_to_text(image), objects=[])],
        }
        headers = {"Content-Type": "application/json"}
        content = dict(data=data, job_type=job_type)
        response = (requests.post(url, data=json.dumps(content), headers=headers)).json()

        return response["objects"], response["plan_actions"], response["raw_output"]
        
        # the response.json() contains the objects(id, label, box), the plan_action(action list)
        # and the raw_output(explanation)


    def transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        try:
            # Initialize the tf2 transform buffer and listener
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)

            # Wait for the transform to become available
            tf_buffer.can_transform(target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(3.0))

            # Transform the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(
                pose,
                tf_buffer.lookup_transform(target_frame, pose.header.frame_id, rospy.Time(0))
            )
            return transformed_pose
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup error: {e}")
        except tf2_ros.ConnectivityException as e:
            rospy.logerr(f"Transform connectivity error: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation error: {e}")
        return None

    def get_3d_position(self, x, y):
        if self.depth_image is None and self.camera_info is None:
            return  # Wait until depth image is received
        depth = (self.depth_image[y, x]/1000)  # Convert to meters
        if np.isnan(depth) or depth == 0:
            rospy.logwarn("Invalid depth at pixel ({}, {})".format(x, y))
            self.recursion +=1
            if self.recursion <= 10:
                self.get_3d_position(x,y)
            else: 
                self.recursion = 0
                return None
            
        # Project the 2D pixel to 3D point in the camera frame
        point_3d = self.camera_model.projectPixelTo3dRay((x, y))
        point_3d = np.array(point_3d) * depth  # Scale the ray by the depth
        pose = PoseStamped()
        pose.header.frame_id = self.camera_model.tf_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point_3d[0]
        pose.pose.position.y = point_3d[1]
        pose.pose.position.z = point_3d[2]
        pose.pose.orientation.w = 1.0

        # transform the pose to the world frame and encapsulate it in a try except block
        try:
            transformed_pose = self.transform_pose(pose, "world")
            if transformed_pose is None:
                rospy.logwarn("Failed to transform the pose to the world frame")
                return
            return transformed_pose
        except Exception as e:
            rospy.logerr(f"Error transforming pose: {e}")
            return None
    
    def color_image_callback(self, msg: Image):
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        pass

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        pass
    
    def camera_info_callback(self, msg: sensor_msgs.msg.CameraInfo):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)
        pass

    # # execute all the actions in the action list one by one here.
    # def execute_actions(self, action_list_suction, action_list_rws):
    #     rospy.loginfo("Sending move preaction goal")
    #     move_preaction_goal = MovePreactionActionGoal()
    #     self.move_preaction_client.send_goal(move_preaction_goal)
    #     self.move_preaction_client.wait_for_result()
    #     move_preaction_result = self.move_preaction_client.get_result()
    #     print("Move preaction result : ", move_preaction_result.result)
        
    #     clean_goal = PickPlaceGoal()
    #     clean_goal.source = PointStamped()
    #     clean_goal.destination = PointStamped()
    #     print("Cleansing the Gripper")
    #     self.clean_client.send_goal(clean_goal)
    #     self.clean_client.wait_for_result()
    #     clean_result = self.clean_client.get_result()
    #     print("clean result : ", clean_result.result)
        
    #     print("Executing actions ...")
        
    #     for action in action_list:
    #         source = action["source_object_position"]
    #         destination = action["target_object_position"]

    #         ### this is for testing ###
    #         source.point.z += 0
    #         destination.point.z += 0

    #         rospy.loginfo("Sending pick and place goal")
    #         pick_place_goal = PickPlaceGoal()
    #         pick_place_goal.source = source
    #         pick_place_goal.destination = destination
    #         self.pick_place_client.send_goal(pick_place_goal)
    #         self.pick_place_client.wait_for_result()
    #         pick_place_result = self.pick_place_client.get_result()
    #         print("Pick and place result : ", pick_place_result.result)
    #         rospy.sleep(0.3)
    #         # x = input("Press Enter to continue")

    #     clean_goal = PickPlaceGoal()
    #     clean_goal.source = PointStamped()
    #     clean_goal.destination = PointStamped()
    #     print("Cleansing the Gripper")
    #     self.clean_client.send_goal(clean_goal)
    #     self.clean_client.wait_for_result()
    #     clean_result = self.clean_client.get_result()
    #     print("clean result : ", clean_result.result)

    #     rospy.loginfo("Sending move preaction goal")
    #     move_preaction_goal = MovePreactionActionGoal()
    #     self.move_preaction_client.send_goal(move_preaction_goal)
    #     self.move_preaction_client.wait_for_result()
    #     move_preaction_result = self.move_preaction_client.get_result()
    #     print("Move preaction result : ", move_preaction_result.result)
    #     print("Done")

def calculate_hex_positions(x: float, y: float, r: float) -> list:
    # Calculate each of the six positions
    x1, y1 = x + r, y
    x2, y2 = x + r * math.cos(math.radians(60)), y - r * math.sin(math.radians(60))
    x3, y3 = x - r * math.cos(math.radians(60)), y - r * math.sin(math.radians(60))
    x4, y4 = x - r, y
    x5, y5 = x - r * math.cos(math.radians(60)), y + r * math.sin(math.radians(60))
    x6, y6 = x + r * math.cos(math.radians(60)), y + r * math.sin(math.radians(60))

    # Return the positions as a list of lists
    return [[x1, y1], [x2, y2], [x3, y3], [x4, y4], [x5, y5], [x6, y6]]


# driver for the central client
if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.1)
    
    # get instruction from the user
    instruction = input("Enter the prompt : ")
    print("Requesting for plan from API ...")
    print("Prompt : ", instruction)
    print("Getting plan ...")

    # make sure both the robots are in preaction
    rospy.loginfo("Sending move preaction goal to left arm")
    move_preaction_goal = MovePreactionActionGoal()
    central_client.left_move_preaction_client.send_goal(move_preaction_goal)
    central_client.left_move_preaction_client.wait_for_result()
    move_preaction_result = central_client.left_move_preaction_client.get_result()
    print("Move preaction result : ", move_preaction_result.result)
    
    rospy.loginfo("Sending move preaction goal to right arm")
    move_preaction_goal = MovePreactionActionGoal()
    central_client.right_move_preaction_client.send_goal(move_preaction_goal)
    central_client.right_move_preaction_client.wait_for_result()
    move_preaction_result = central_client.right_move_preaction_client.get_result()
    print("Move preaction result : ", move_preaction_result.result)

    rospy.sleep(0.2)
    # get plan, 2d object positions, raw output
    objects, plan_actions, raw_output = central_client.get_plan(instruction=instruction)

    # validate the response
    if (objects is None or plan_actions is None or 
        raw_output is None or len(objects) == 0 or 
        len(plan_actions) == 0 or raw_output == ""):
        rospy.logerr("API response is None")
        exit()
    
    # display the output
    print("Objects detected : ", len(objects))
    for object in objects:
        print(object["label"])
    for plan in plan_actions:
        print(plan)
    print("Explanation : ",raw_output)
    
    # display the image with the objects and the bounding boxes
    image = Image.fromarray(
        cv2.cvtColor(
            central_client.color_image, cv2.COLOR_BGR2RGB
            )
        )
    
    draw = ImageDraw.Draw(image)
    
    for object in objects:
        draw.rectangle(object["box"], outline="red", width=2)
        position = (int(object["box"][0]), int(object["box"][1]) - min(image.size) // 32)
        draw.rectangle(draw.textbbox(position, object["label"]), fill="red")
        draw.text(position, object["label"], fill="white")

    image.save("bounding boxes.png")


    # process plan actions and compute the 3d positions
    print("Processing LLM actions ..... ")
    
    objects_3d = []
    for object in objects:
        object_3d = {}
        x = int((object["box"][0] + object["box"][2])//2)
        y = int((object["box"][1] + object["box"][3])//2)
        pose = PoseStamped()
        pose = central_client.get_3d_position(x, y)
        label = object["label"]
        label = label.split(" ")
        object_3d["id"] = int(label[0])
        # object_3d["label"] = label[1] + " " + label[2]
        # loop through the label to get the full label
        object_3d["label"] = ""
        for i in range(1, len(label)):
            object_3d["label"] += label[i] + " "
        object_3d["pose"] = pose
        
        print("object : ", object_3d["label"])
        objects_3d.append(object_3d)

    for action in plan_actions:
        action["source_object_id"] = int(action["source_object_id"])
        action["target_object_id"] = int(action["target_object_id"])

    
    print("#################################################")
    print("Plan actions : ")
    print(plan_actions)
    print("#################################################")
    print("Objects 3d : ")
    print(objects_3d)
    print("#################################################")
    
    # variable formats : 
    # object_3d = {"id":1, "label":"apple", "pose":<pose>:PoseStamped}
    # action = {"action_type":"pick_and_place", "source_object_id":1, "target_object_id":2}

    plate_pose =  None

    # get the plate pose
    for object in object_3d:
        if object["label"] == "empty white plate":
            plate_pose = object["pose"] 
    
    # get 6 offsetted plate pose from the original, r=3cm
    hex_positions = calculate_hex_positions(plate_pose.pose.position.x,plate_pose.pose.position.y,0.03)
    hex_poses = []
    for i in range(len(hex_positions)):
        hex_pose = PoseStamped()
        hex_pose.pose.position.x = hex_positions[i][0]
        hex_pose.pose.position.y = hex_positions[i][1]
        hex_pose.pose.position.z = plate_pose.pose.position.z
        hex_poses.append(hex_pose)

    print("Plate position : ", plate_pose)
    print("###################")
    print("Plate hex poses : ", hex_poses)
    print("###################")

    set_suction_objects = set("pineapple slices","cheese slices")

    action_list_suction = []
    action_list_rws = []

    i = 0
    for action in plan_actions :
        action_parsed = {
            "action_type": action["action_type"],
            "source_object_position": objects_3d[action["source_object_id"]]["position"],
            "target_object_position": hex_poses[i]
        }
        if objects_3d[action["source_object_id"]]["label"] in set_suction_objects:
            action_list_suction.append(action_parsed)
        else :
            action_list_rws.append(action_parsed)
        i+=1
    
    print("Actions for suction : ")
    print(action_list_suction)
    print("###################")
    print("Actions for rws : ")
    print(action_list_rws)
    print("###################")

    # user validation of the action list and proceed to execute the actions
    input("Press Enter to continue ...")
    
    # execution of the actions
    # central_client.execute_actions(action_list_rws=action_list_rws,action_list_suction=action_list_suction)
