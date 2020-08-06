#!/usr/bin/env python
import os
import sys
import re
import socket
import getpass
import rospy
import message_filters
from sensor_msgs.msg import Range

# if you are on the robotcar and you have done the whole installation from here:
# https://github.com/Michdo93/robotcar/
# you can also import from /home/$USER/robotcar/filter/kalman/SimpleKalmanFilter-Python

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/catkin_ws/src/robotcar_sensorfusion_examples/src'))
sys.path.insert(0, env)

from SimpleKalmanFilter import SimpleKalmanFilter

class LeftDistance(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.ultrasonicFrontSub = message_filters.Subscriber(self.robot_host + '/ultrasonic/front/left/distance', Range)
        self.ultrasonicRearSub = message_filters.Subscriber(self.robot_host + '/ultrasonic/rear/left/distance', Range)

        ts = message_filters.ApproximateTimeSynchronizer([self.ultrasonicFrontSub, self.ultrasonicRearSub], 10, 0.1)
        ts.registerCallback(self.callback)

        # Initialize message variables.
        self.ultrasonicFrontData = 0
        self.ultrasonicRearData = 0

        self.KFultrasonicFront = SimpleKalmanFilter(2, 2, 0.01)
        self.KFultrasonicRear = SimpleKalmanFilter(2, 2, 0.01)

        self.ultrasonicFrontSensorKalman = 0.0
        self.ultrasonicRearSensorKalman = 0.0

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.ultrasonicFrontSub = message_filters.Subscriber(self.robot_host + '/ultrasonic/front/left/distance', Range)
        self.ultrasonicRearSub = message_filters.Subscriber(self.robot_host + '/ultrasonic/rear/left/distance', Range)

        ts = message_filters.ApproximateTimeSynchronizer([self.ultrasonicFrontSub, self.ultrasonicRearSub], 10, 0.1)
        ts.registerCallback(self.callback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.ultrasonicFrontSub.unregister()
        self.ultrasonicRearSub.unregister()

    def applyKF(self):
        self.ultrasonicFrontSensorKalman = self.KFultrasonicFront.updateEstimate(self.ultrasonicFrontData.range)
        self.ultrasonicRearSensorKalman = self.KFultrasonicRear.updateEstimate(self.ultrasonicRearData.range)

        kalman = (self.ultrasonicFrontSensorKalman + self.ultrasonicRearSensorKalman)/2

        filterMsg = "ultrasonicFrontSensorKalman: %s and ultrasonicRearSensorKalman: %s" % (self.ultrasonicFrontSensorKalman, self.ultrasonicRearSensorKalman)
        kalmanMsg = "Distance Kalman Filter: %s" % kalman

        rospy.loginfo(filterMsg)
        rospy.loginfo(kalmanMsg)

    def callback(self, ultrasonicFrontData, ultrasonicRearData):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.ultrasonicFrontData = ultrasonicFrontData
        self.ultrasonicRearData = ultrasonicRearData
        
        ultrasonicFrontMsg = "Got type %s with FoV %s and Min-Range %s and Max-Range %s and measured Range %s" % (self.ultrasonicFrontData.radiation_type, self.ultrasonicFrontData.field_of_view, self.ultrasonicFrontData.min_range, self.ultrasonicFrontData.max_range, self.ultrasonicFrontData.range)
        ultrasonicRearMsg = "Got type %s with FoV %s and Min-Range %s and Max-Range %s and measured Range %s" % (self.ultrasonicRearData.radiation_type, self.ultrasonicRearData.field_of_view, self.ultrasonicRearData.min_range, self.ultrasonicRearData.max_range, self.ultrasonicRearData.range)

        #rospy.loginfo("leftFrontUltrasonic: " + ultrasonicFrontMsg)
        #rospy.loginfo("leftRearUltrasonic: " + ultrasonicRearMsg)

        self.applyKF()

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_LeftDistanceSubscriber"
    rospy.init_node(node_name, anonymous=False)
    
    left = LeftDistance("robotcar")
    
    # Go to the main loop
    try:
        left.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        left.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill "+ node_name)

        
        print("Node stopped")