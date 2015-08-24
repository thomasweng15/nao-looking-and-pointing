#!/usr/bin/env python

import sys
import argparse
import math
from numpy import *
import rospy
import time
from lego import Lego
from std_msgs.msg import String
from kinect2_pointing_recognition.msg import SkeletonInfo, ObjectsInfo, FaceInfo, SpeechInfo
from nao_looking_and_pointing.msg import HumanBehavior

def dot_prod(a, b):
	return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]

def magn(a):
	return math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])

class ComputeAttention():
	def __init__(self, max_people, point_aperture, look_aperture, touch_distance):
		self.node_name = 'compute_attention'
		rospy.init_node(self.node_name)

		self.behaviorpub = rospy.Publisher('human_behavior', HumanBehavior)

		self.point_aptr = point_aperture
		self.look_aptr = look_aperture
		self.touch_dist = touch_distance

		self.skeletons = Skeletons(max_people)
		self.faces = Faces(max_people)
		self.objects = Objects()
		self.speech = Speech()

		self.rate = rospy.Rate(5) # 5hz, or 5 per second

		self.head_target = None
		self.leftarm_target = None
		self.rightarm_target = None

	def leftarm_pointing(self, obj):
		if self.leftarm_target == obj:
			return True
		return False

	def rightarm_pointing(self, obj):
		if self.rightarm_target == obj:
			return True
		return False

	def head_pointing(self, obj):
		if self.head_target == obj:
			return True
		return False

	def face_vs_obj_angle(self, person_id, object_id):
		face_orientation = self.faces.face_orientation_vec(person_id)
		head_to_obj = self.head_to_obj_vec(person_id, object_id)

		dotprod = dot_prod(face_orientation, head_to_obj)
		if dotprod != 0.0:
			cosine = dotprod / (magn(face_orientation) * magn(head_to_obj))
			angle = math.acos(cosine)
			return angle

	def head_to_obj_vec(self, person_id, object_id):
		if list(self.skeletons.array[person_id].head) == [0.0, 0.0, 0.0]:
			return [0.0, 0.0, 0.0]
		return array(self.skeletons.array[person_id].head) - self.objects.objdict[object_id].pos

	def point_vs_obj_angles(self, person_id, object_id):
		head_to_handtips = self.skeletons.head_to_handtip_vecs(person_id)
		handtips_to_obj = self.handtip_to_obj_vecs(person_id, object_id)

		left_point = head_to_handtips[0]
		left_handtip_to_obj = handtips_to_obj[0]
		left_dotprod = dot_prod(left_point, left_handtip_to_obj)
		left_point_vs_obj_angle = None

		if left_dotprod != 0.0:
			left_cosine = left_dotprod / (magn(left_point) * magn(left_handtip_to_obj))
			left_point_vs_obj_angle = math.acos(left_cosine)

		right_point = head_to_handtips[1]
		right_handtip_to_obj = handtips_to_obj[1]
		right_dotprod = dot_prod(right_point, right_handtip_to_obj)
		right_point_vs_obj_angle = None

		if right_dotprod != 0.0:
			right_cosine = right_dotprod / (magn(right_point) * magn(right_handtip_to_obj))
			right_point_vs_obj_angle = math.acos(right_cosine)

		return [left_point_vs_obj_angle, right_point_vs_obj_angle]

	def handtip_to_obj_vecs(self, person_id, object_id):
		if list(self.skeletons.array[person_id].handTipLeft) == [0.0, 0.0, 0.0]:
			return [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
		left = self.objects.objdict[object_id].pos - array(self.skeletons.array[person_id].handTipLeft)
		right = self.objects.objdict[object_id].pos - array(self.skeletons.array[person_id].handTipRight)
		return [left, right]

	def is_touching(self, person_id, object_id):
		handtips_to_obj = self.handtip_to_obj_vecs(person_id, object_id)
		if not any(handtips_to_obj[0]) and not any(handtips_to_obj[1]):
			return [None, None]
		return [magn(handtips_to_obj[0]), magn(handtips_to_obj[1])]

	def run(self):
		""" Watch for pointing and gaze actions, then publish them. """
		while not rospy.is_shutdown():
			# Track results
			effectorlist = []
			targetlist = []

			# get person id of the person in frame
			person_id = None
			for i in range(0, self.skeletons.max_people):
				if list(self.skeletons.array[i].head) != [0.0,0.0,0.0]:
					person_id = i
			if person_id == None:
				rospy.loginfo("No one is detected in the frame.")
				time.sleep(1)
				continue

			# Boolean and Radians of Head vs. Object
			rospy.loginfo("%s", "Booleans and Radians of Head vs. Object")
			smallest_angle = 1000
			head_target = None
			for j in range(0, len(self.objects.objdict)):
				looking_angle = self.face_vs_obj_angle(person_id, j)
				threshold = looking_angle is not None and looking_angle < self.look_aptr / 2
				if threshold and looking_angle < smallest_angle:
					smallest_angle = looking_angle
					head_target = j
				rospy.loginfo("%s\t%s", threshold, looking_angle)
			if head_target:
				effectorlist.append("head")
				targetlist.append(head_target)
			rospy.loginfo("Looking at %s", head_target)

			# Boolean and Radians of Point vs. Object
			rospy.loginfo("%s", "Booleans and Radians of Point vs. Object")
			r_smallest_angle = 1000
			r_target_object = None
			l_smallest_angle = 1000
			l_target_object = None
			for j in range(0, len(self.objects.objdict)):
				pointing_angles = self.point_vs_obj_angles(person_id, j)
				pointing_threshold = [angle is not None and angle < self.point_aptr / 2 for angle in pointing_angles]
				if pointing_threshold[0] and pointing_angles[0] < l_smallest_angle:
					l_smallest_angle = pointing_angles[0]
					l_target_object = j
				if pointing_threshold[1] and pointing_angles[1] < r_smallest_angle:
					r_smallest_angle = pointing_angles[1]
					r_target_object = j
				rospy.loginfo("%s\t%s", pointing_threshold, pointing_angles)
			if r_target_object:
				effectorlist.append("rightarm")
				targetlist.append(r_target_object)
			if l_target_object:
				effectorlist.append("leftarm")
				targetlist.append(l_target_object)
			rospy.loginfo("Right arm pointing at %s", r_target_object)
			rospy.loginfo("Left arm pointing at %s", l_target_object)

			rospy.loginfo("-----------")

			# Publish a HumanBehavior message if there is information to send
			if effectorlist:
				assert len(effectorlist) == len(targetlist)
				behaviorMsg = HumanBehavior()
				behaviorMsg.effector = effectorlist
				behaviorMsg.target = targetlist

				self.behaviorpub.publish(behaviorMsg)

			self.rate.sleep()



class Skeletons():
	def __init__(self, max_people):
		self.max_people = max_people
		self.array = [SkeletonInfo()] * max_people
		rospy.Subscriber('/skeleton_info', SkeletonInfo, self.skeleton_callback)

	def skeleton_callback(self, msg):
		self.array[int(msg.person_id)] = msg
	
	def head_to_handtip_vecs(self, person_id):
		if list(self.array[person_id].head) == [0.0, 0.0, 0.0]:
			return [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
		left = array(self.array[person_id].handTipLeft) - self.array[person_id].head
		right = array(self.array[person_id].handTipRight) - self.array[person_id].head
		return [left, right]

class Faces():
	def __init__(self, max_people):
		self.max_people = max_people
		self.array = [FaceInfo()] * max_people
		rospy.Subscriber('/face_info', FaceInfo, self.face_callback)

	def face_callback(self, msg):
		self.array[int(msg.person_id)] = msg

	def face_orientation_vec(self, person_id):
		if self.array[person_id].yaw == -1.0 and self.array[person_id].pitch == -1.0: 
			return [0.0, 0.0, 0.0]
		yaw_rad = math.radians(self.array[person_id].yaw)
		pitch_rad = math.radians(self.array[person_id].pitch)
		return [math.sin(yaw_rad), math.sin(pitch_rad), math.cos(yaw_rad)]

class Objects():
	def __init__(self):
		self.objdict = dict()

		rospy.Subscriber('/objects_info', ObjectsInfo, self.objects_callback)

	def objects_callback(self, objectMsg):
		obj_id = int(objectMsg.object_id)

		if obj_id in self.objdict:
			# if the object location has changed, update it
			if not objectMsg.pos == self.objdict[obj_id].loc:
				self.objdict[obj_id].loc = objectMsg.pos
			else:
				# if this is a new object, add it to objdict
				o = Lego(obj_id,   		   # ID number
					objectMsg.pos,         # 3D position
					objectMsg.color_upper, # upper RGB color threshold
					objectMsg.color_lower, # lower RGB color threshold
					'')                    # descriptor words
				self.objdict[obj_id] = o

class Speech():
	def __init__(self):
		self.words = None
		self.timestamp = rospy.get_rostime()
		rospy.Subscriber('speech_info', SpeechInfo, self.speech_callback)

	def speech_callback(self, msg):
		self.words = msg.speech
		self.timestamp = rospy.get_rostime()
	

if __name__ == "__main__":
	sys.argv = rospy.myargv(argv=sys.argv)

	parser = argparse.ArgumentParser(description="Listen for human nonverbal behavior")
	parser.add_argument('--max_people',
	    help='int representing the max number of skeletons the Kinect will return',
	    dest='max_people',
	    default='6',
	    type=int)
	parser.add_argument('--pointsize',
		help='pointing aperture size in radians',
		dest='point_aperture',
		default='0.7',
		type=float)
	parser.add_argument('--looksize',
		help='looking aperture size in radians',
		dest='look_aperture',
		default='3.0',
		type=float)
	parser.add_argument('--touchdist',
		help='distance in meters from finger to object that counts as a touch',
		dest='touch_distance',
		default='0.1',
		type=float)

	args = parser.parse_args()
	people = args.max_people
	point = args.point_aperture
	look = args.look_aperture
	touch = args.touch_distance

	ca = ComputeAttention(people, point, look, touch)
	ca.run()
