Dependencies:
- numpy python library
- libjsoncpp C++ library
- fourbythree_msgs ROS package (provided by Tekniker)
- readchar python library
- OpenNI ROS package (provided by Tekniker)
- OpenNI Tracker ROS package (provided by Tekniker)

PREPARE KINECT
1. Make the Kinect is on and connected to the computer
2. Open a new terminal and run the Kinect:
roslaunch openni_launch openni.launch camera:=openni
3. Open New Terminal and run the human skeleton tracker:
roslaunch openni_tracker openni_skeleton_tracker.launch
4. The human needs to stand up in front of the camera. Wait for the frame in R-Viz to be available. Check the human index and remember it for the next section. For example, if the left elbow for example is detected as ‘left_elbow_2’, then the human index is ‘2’. Make sure to check which arm is assigned as “left” in the R-Viz. It is possible for your right arm to be assigned as “left”. Only “left” arm can be used both in the Stiffness Adjustment and Ergonomics part.

ERGONOMICS
1. Open New terminal and run the ergonomics code:
roslaunch kcl_ergonomics launch_ergonomics.launch human:=1
The argument 'human' is assigned according to the human index in R-Viz.
2. The worker needs to be in calibration position (lower arm and upper arm form 90 degrees angle). After that, in the same terminal, press ‘z’ to starts the calibration and the algorithm is ready. Whenever you want to go back to calibration mode, press ‘x’. The Baxter's hand will move automatically if the ergonomics of the worker is low, such that it tries to improve the ergonomics of the worker.
3. In order the see the RULA score, subscribe to this following type:
/fourbythree_topics/ergonomics/rula_score
