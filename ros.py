import rospy
from naoGestures import NaoGestures
from std_msgs.msg import String
from kinect2_pointing_recognition.msg import ObjectsInfo

def parse_objects_callback(objects_msg):
	if objects_msg.object_id == '0': # grab the first object, as example
		print objects_msg.pos

rospy.init_node('nao_gestures')
naoGestures = NaoGestures()
sub = rospy.Subscriber('/objects_info', ObjectsInfo, parse_objects_callback)
rospy.spin()