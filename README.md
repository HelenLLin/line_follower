# line_follower PA

The line_follower PA uses OpenCV with ROS images to make the robot follow a line. The code marks out where the line is and then clears a area near the line.
It then marks the center of the line using a red dot. If the dot is off-center, the robot uses a PID error value to calculate it's angular Twist and reorientate itself.
The code makes use of the ability to convert between OpenCV and ROS compressed image message types.

# Running the code

1. Open a terminal and start roscore ```$ roscore```
2. Open a new tab and open up the turtlebot world. This PA uses the waffle model since it has a camera. ```$ roslaunch prrexamples linemission.launch model:=waffle```
3. Open another tab and start line_follower.py ```$ rosrun line_follower line_follower.py```
4. Watch the robot follow the lines!
