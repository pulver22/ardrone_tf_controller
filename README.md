# ardrone_tf_controller
Position controller the AR Drone detecting an ARMarker

#Launch
To run: `rosrun ardrone_tf_controller autopilot.launch`


#Changelog

##Version 1.0.0
- Navigation with two fixed camera (a front one and a bottom one) implemented and validation experiments performed


##Version 0.3.0
- Tf lookup from transformation between marker's and bottomcam's frame and provide it as next goal to be reached.

##Version 0.2.0
- Tf lookup from transformation between marker's and frontcam's frame and provide it as next goal to be reached

##Version 0.1.0
- Added a subscriber to marker frame wrt camera's one
