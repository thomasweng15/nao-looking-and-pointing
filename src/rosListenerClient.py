from wsgiref.simple_server import make_server
#from ws4py.client.threadedclient import WebSocketClient
from ws4py.websocket import WebSocket
from ws4py.server.wsgirefserver import WSGIServer, WebSocketWSGIRequestHandler
from ws4py.server.wsgiutils import WebSocketWSGIApplication
import rospy
from std_msgs.msg import String

class RosListenerServer(WebSocket):
	"""
	A WebSocket server that sends messages from ROS topics.
	"""

	def opened(self):
		self.send("Opened")
		print "ROS Listener Server connected."

		# Listen for script status messages
		rospy.init_node("Websocket_listener")
		self.script_listener = rospy.Subscriber(
			"/script_status", String, self.sendRosMessage)

		self.send("Test message")
		self.send("Test message")

	def received_message(self, m):
		print "Received: " + str(m)

	def sendRosMessage(self, msg):
		""" Send the ROS message along to the server. """
		#if msg.data == "ResetBlocks":
		print "Sending message: " + msg.data
		self.send(msg.data)


if __name__ == '__main__':
	server = make_server('', 8080, 
						server_class	= WSGIServer, 
						handler_class	= WebSocketWSGIRequestHandler, 
						app				= WebSocketWSGIApplication(
											handler_cls=RosListenerServer))
	server.initialize_websockets_manager()
	server.serve_forever()
