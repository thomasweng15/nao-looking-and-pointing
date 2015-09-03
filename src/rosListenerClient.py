from wsgiref.simple_server import make_server
#from ws4py.client.threadedclient import WebSocketClient
from ws4py.websocket import WebSocket, EchoWebSocket
from ws4py.server.wsgirefserver import WSGIServer, WebSocketWSGIRequestHandler
from ws4py.server.wsgiutils import WebSocketWSGIApplication
import rospy
from std_msgs.msg import String

#windowsIP = "ws://10.0.0.26:9000/"

class EchoWebSocket1(WebSocket):
	def opened(self):
		self.send("Opened")

	def received_message(self, m):
		print m
		if m == "ResetBlocks":
			print "Reset blocks"
		self.sendRosMessage("ResetBlocks")

	def sendRosMessage(self, msg):
		""" Send the ROS message along to the server. """
		print "Send reset blocks"
		if msg.data == "ResetBlocks":
			print "Sending message: " + msg.data
			self.send(msg.data)

class ROSListenerClient(EchoWebSocket):
	def __init__(self, url):
		EchoWebSocket.__init__(self, url)

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
		if m == "ResetBlocks":
			print "Reset blocks"
		self.sendRosMessage("ResetBlocks")

	def sendRosMessage(self, msg):
		""" Send the ROS message along to the server. """
		print "Send reset blocks"
		if msg.data == "ResetBlocks":
			print "Sending message: " + msg.data
			self.send(msg.data)


if __name__ == '__main__':
	server = make_server('', 9000, server_class=WSGIServer, 
		handler_class=WebSocketWSGIRequestHandler, 
		app=WebSocketWSGIApplication(handler_cls=EchoWebSocket1))
	server.initialize_websockets_manager()
	#server = WSGIServer(('localhost', 9000), WebSocketWSGIApplication(handler_cls=ROSListenerClient))
	#ws = ROSListenerClient(windowsIP)
	#ws.connect()
	server.serve_forever()

	#ws.close()