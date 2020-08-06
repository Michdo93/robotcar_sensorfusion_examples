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

class FrontDistance(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.irSub = message_filters.Subscriber(self.robot_host + '/infrared/front/distance', Range)
        self.tofSub = message_filters.Subscriber(self.robot_host + '/time_of_flight/front/distance', Range)
        self.ultrasonicSub = message_filters.Subscriber(self.robot_host + '/ultrasonic/front/distance', Range)

        ts = message_filters.ApproximateTimeSynchronizer([self.irSub, self.tofSub, self.ultrasonicSub], 10, 0.1)
        ts.registerCallback(self.callback)

        # Initialize message variables.
        self.irData = 0
        self.tofData = 0
        self.ultrasonicData = 0

        self.KFir = SimpleKalmanFilter(2, 2, 0.01)
        self.KFtof = SimpleKalmanFilter(2, 2, 0.01)
        self.KFultrasonic = SimpleKalmanFilter(2, 2, 0.01)

        self.irSensorKalman = 0.0
        self.tofSensorKalman = 0.0
        self.ultrasonicSensorKalman = 0.0

        self.enable = False

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.irSub = message_filters.Subscriber(self.robot_host + '/infrared/front/distance', Range)
        self.tofSub = message_filters.Subscriber(self.robot_host + '/time_of_flight/front/distance', Range)
        self.ultrasonicSub = message_filters.Subscriber(self.robot_host + '/ultrasonic/front/distance', Range)

        ts = message_filters.ApproximateTimeSynchronizer([self.irSub, self.tofSub, self.ultrasonicSub], 10, 0.1)
        ts.registerCallback(self.callback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.irSub.unregister()
        self.tofSub.unregister()
        self.ultrasonicSub.unregister()

    def applyKF(self):
        self.irSensorKalman = self.KFir.updateEstimate(float(self.irData.range))
        self.tofSensorKalman = self.KFtof.updateEstimate(self.tofData.range)
        self.ultrasonicSensorKalman = self.KFultrasonic.updateEstimate(self.ultrasonicData.range)

        kalman = (self.irSensorKalman + self.tofSensorKalman + self.ultrasonicSensorKalman)/3

        filterMsg = "irSensorKalman: %s and tofSensorKalman: %s and ultrasonicSensorKalman: %s" % (self.irSensorKalman, self.tofSensorKalman, self.ultrasonicSensorKalman)
        kalmanMsg = "Distance Kalman Filter: %s" % kalman

        rospy.loginfo(filterMsg)
        rospy.loginfo(kalmanMsg)

    def callback(self, irData, tofData, ultrasonicData):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.irData = irData
        self.tofData = tofData
        self.ultrasonicData = ultrasonicData
        
        irMsg = "Got type %s with FoV %s and Min-Range %s and Max-Range %s and measured Range %s" % (self.irData.radiation_type, self.irData.field_of_view, self.irData.min_range, self.irData.max_range, self.irData.range)
        tofMsg = "Got type %s with FoV %s and Min-Range %s and Max-Range %s and measured Range %s" % (self.tofData.radiation_type, self.tofData.field_of_view, self.tofData.min_range, self.tofData.max_range, self.tofData.range)
        ultrasonicMsg = "Got type %s with FoV %s and Min-Range %s and Max-Range %s and measured Range %s" % (self.ultrasonicData.radiation_type, self.ultrasonicData.field_of_view, self.ultrasonicData.min_range, self.ultrasonicData.max_range, self.ultrasonicData.range)

        #rospy.loginfo("frontInfrared: " + irMsg)
        #rospy.loginfo("frontToF: " + tofMsg)
        #rospy.loginfo("frontUltrasonic: " + ultrasonicMsg)

        self.applyKF()

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_FrontDistanceSubscriber"
    rospy.init_node(node_name, anonymous=False)
    
    front = FrontDistance("robotcar")
    
    # Go to the main loop
    try:
        front.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        front.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill "+ node_name)

        
        print("Node stopped")