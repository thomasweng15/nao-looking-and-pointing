import rospy
from naoGestures import NaoGestures
from std_msgs.msg import String
from kinect2_pointing_recognition.msg import ObjectsInfo

class GestureController():
	def __init__(self):
		rospy.init_node('nao_gestures')
		self.sub = rospy.Subscriber('/objects_info', ObjectsInfo, self.parseObjectsCallback)
		self.naoGestures = NaoGestures()
		self.doAction = True

	def parseObjectsCallback(self, objectMsg):
		if objectMsg.object_id == '0': # grab the first object, as example
			if self.doAction:
				coords = self.convertCoords(objectMsg.pos)
				self.naoGestures.doGesture('lookandpoint', coords)
				self.doAction = False

	def convertCoords(self, kinectCoords):
		'''
		The Kinect and the Nao use define coordinate systems differently. 
		This function converts Kinect coordinates to Nao coordinates. 
		'''
		# TODO transform origin from kinect to nao torso
		return [kinectCoords[2], kinectCoords[0], kinectCoords[1]]

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	gs = GestureController()
	gs.run()
