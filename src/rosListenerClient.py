from ws4py.client.threadedclient import WebSocketClient
import rospy
from std_msgs.msg import String

windowsIP = "ws://localhost:9000/"

class ROSListenerClient(WebSocketClient):
	def __init__(self,url):
		WebSocketClient.__init__(self, url)

		# Listen for script status messages
		rospy.init_node("Websocket_listener")
		self.script_listener = rospy.Subscriber(
			"/script_status", String, self.sendRosMessage)

	def opened(self):
		self.send("Opened")

	def closed(self, code, reason=None):
		print "Socket closed", code, reason

	def received_message(self, m):
		print m

	def sendRosMessage(self, msg):
		""" Send the ROS message along to the server. """
		if msg.data == "ResetBlocks":
			print "Sending message: " + msg.data
			self.send(msg.data)


if __name__ == '__main__':
	ws = ROSListenerClient(windowsIP)
	ws.connect()
	ws.run_forever()

	ws.close()