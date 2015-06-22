import rospy
from naoGestures import NaoGestures
from std_msgs.msg import String
from kinect2_pointing_recognition.msg import ObjectsInfo

class GestureController():
	def __init__(self):
		rospy.init_node('nao_gestures')
		self.sub = rospy.Subscriber('/objects_info', ObjectsInfo, self.parse_objects_callback)
		self.naoGestures = NaoGestures()
		self.doAction = True

	def parse_objects_callback(self, object_msg):
		if object_msg.object_id == '1': # grab the first object, as example
			if self.doAction:
				# TODO translate kinect coordinates to nao torso coordinates
				self.naoGestures.doGesture('lookandpoint', object_msg.pos)
				self.doAction = False

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	gs = GestureController()
	gs.run()