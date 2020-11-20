# robotcar_sensorfusion_examples

Gives an exampe how you can use sensor fusion with the [RobotCar](https://github.com/Michdo93/robotcar). It uses the Kalman-Filter implementation of [SimpleKalmanFilter](https://github.com/Michdo93/SimpleKalmanFilter-Python). At first you have to make sure that the roscore is running and the [robotcar-pkg}(https://github.com/Michdo93/robotcar-pkg) is publishing the sensor informations.

The String variable `robot_host` uses the hostname of one Robotcar. As example it could be `robotcar`.

This package could be used as blue print for some ADAS which uses multiple sensor informations. To receive multiple sensor informations in one subscriber callback method you have to use the TimeSynchronizer or ApproximateTimeSynchronizes from [message_filters](http://wiki.ros.org/message_filters).

## FrontDistanceSubscriber Node

It subscribes the sensor informations from the front sensors. Following adresses are used:

                  Topic Adress                |      Message Type
--------------------------------------------- | --------------------
robot_host + /infrared/front/distance         | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)
robot_host + /time_of_flight/front/distance   | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)
robot_host + /ultrasonic/front/distance       | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)

## LeftDistanceSubscriber Node

It subscribes the sensor informations from the left sensors. Following adresses are used:

                  Topic Adress                |      Message Type
--------------------------------------------- | --------------------
robot_host + /ultrasonic/front/left/distance  | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)
robot_host + /ultrasonic/rear/left/distance   | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)

## RightDistanceSubscriber Node

It subscribes the sensor informations from the right sensors. Following adresses are used:


                  Topic Adress                |      Message Type
--------------------------------------------- | --------------------
robot_host + /ultrasonic/front/right/distance | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)
robot_host + /ultrasonic/rear/right/distance  | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)

## RearDistanceSubscriber Node

It subscribes the sensor informations from the rear sensors. Following adresses are used:


                  Topic Adress                |      Message Type
--------------------------------------------- | --------------------
robot_host + /infrared/rear/distance          | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)
robot_host + /time_of_flight/rear/distance    | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)
robot_host + /ultrasonic/rear/distance        | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)
