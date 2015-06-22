# nao-looking-and-pointing
This project is a supplement to the [kinect2-pointing-recognition](https://github.com/thomasweng15/kinect2_pointing_recognition) repository, to allow the Nao robot to point to and look at objects identified and located by the Kinect. 

The two technologies are joined by ROS: the Kinect publishes object coordinates onto rostopics, and the code in this repo subscribes to the relevant rostopics to grab the coordinates. Some transformation is required. 

Naoqi version 2.1
Choregraphe version 2.1.2

`naoGestures.py` contains the `NaoGestures()` class, which controls Nao looking and pointing behavior. Both looking and pointing can be performed at the same time, or independently. 

`main.py` is an example of connecting ROS with the `NaoGestures()` class, demonstrating how the class can accept coordinates published on rostopics. 
