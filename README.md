# Instructions to run the demo

1. connect to the ip adress of the robot using firefox. On truyere, this
   address is 172.17.1.3.

2. release the brakes,
3. activate FCI,
4. in terminal 1, in this directory
   ```bash
   roslaunch robot.launch arm_id:=panda2 robot_ip:=172.17.1.3
   ```
5. to start the camera
   ```bash
   roslaunch realsense2_camera rs_camera.launch
   ```
