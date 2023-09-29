# Calibration

This file explains how to
  - plan, execute motions that move the camera in front of a chessboard,
  - collect eye to hand calibration data,
  - compute and save hand eye calibration parameters

1. start the robot, the camera and the demo as described in the main README.md.

2. start hppcorbaserver and gepetto-gui as described in the main README.md

3. run `calibration.py` script in a terminal

  ```
  cd agimus-demos/franka/manipulation
  python -i calibration.py
  ```
  This should compute some paths going to configurations where the robot
  looks at the chessboard.
  
  In the same terminal, display the robot and environment:
  
  ```
  >>> v = vf.createViewer()
  ```
  
4. place the chessboard as shown in `gepetto-gui`.

5. in a new terminal,

  ```
  python -i run_calibration.py
  ```
