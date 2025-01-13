import rospy
from open_set_object_detection_msgs.srv import GetObjectLocations, GetObjectLocationsRequest, GetObjectLocationsResponse


rospy.init_node("secondary_perception_sample_client")
service_client = rospy.ServiceProxy("secondary_perception_azure",GetObjectLocations)
rospy.sleep(0.01)
msg = GetObjectLocationsRequest()

# TEXT_PROMPT = "round pineapple slices"
TEXT_PROMPT = "round cheese slices"
# TEXT_PROMPT = "detect empty white plate"
# TEXT_PROMPT = "detect orange yellow scrambled eggs"
# TEXT_PROMPT = "detect white rice in a bowl"
# TEXT_PROMPT = "detect green cake"
# TEXT_PROMPT = "detect white dumplings"

msg.prompt.data = TEXT_PROMPT
response = service_client(msg)
print(response.result.object_position[0].pose)