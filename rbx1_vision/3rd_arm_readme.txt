Following is the instructions for running the simple vision control of 3rd arm

1. Run the command: roslaunch freenect_launch freenect-registered-xyzrgb.launch to run the driver of kinect under ROS

2. Better to do calibrations following the ROS wiki Instructions using printed checkerboard for rgb camera or ir camera.

3. Open a new command line window and run: roslaunch rbx1_vision camshift.launch, Then select the object (color blob) you want to track in the rgb image. If succeed, you could see a green rotated rectangle box around the object. Also the box is updating very frequently.

4. Open a new command line window and run: roslaunch rbx1_vision camshift2.launch. Almost same as 3. Well select the 3rd arm finger(or hands) with some color blob on it. 

5. Open a new command line window and run: roslaunch rbx1_vision pos_calculator.launch. Then it will publish the 3d vector from 3rd arm to the object on the ros topic /direction. The vector is in unit meters.
