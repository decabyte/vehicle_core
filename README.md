vehicle_core
============

This repo holds the ROS package of the Autonomous Vehicle Software (AVS) used with OSL vehicles. This package is
meant to be python importable from other scripts and packages. It provides a consolidated set of modules, nodes,
configuration and launch files to run basic missions with one of the OSL's vehicles.

Guidelines
----------

Before attempting any modification to this repo please make sure thar:
  - you are working on an up-to-date version of the `master` branch
  - the previous working version of the repo has been tagged (`git tag --list` to check the tags)
  - you have considered the option of creating a new branch for your feature (`git checkout -b <branch_name>` to create a new branch), after all this is the recommended approach!
  - you know what are you doing!
  
Initially this repo is providing packages using the `rosbuild` build system until the OSL vehicles are migrated to an
 upgraded version of the ROS. Later the rosbuild support is going to be dropped and the master branch will offer a
 catkinized package format. The software development follows a couple of principles that have been proven useful
 during the early development of this project like [Semver][semver], for semantic versioning of releases and tags,
 [KISS][kiss], as a general guideline to prevent the commit of _huge_ files and _beast_ modules, and, finally,
 [TDD][tdd], as a principle for testing your code if you want to rely on a more pragmatic approach.

Requirements
------------
  - A valid ROS installation (version Hydro, Indigo or later is suggested)
  - ROS joy package from ROS Joystick Drivers (`$ sudo apt-get install ros-<VERSION>-joy`)
  - Python 2.7+
  - Numpy 1.8+
  - Scipy 0.13+

Run Simulator
---

1) Run nav simulator
  ```
  roslaunch vehicle_core nav_sim.launch
  ```

2) Run vehicle controller
  ```
  roslaunch vehicle_core pilot_sim.launch
  ```

3) Run visualization
  ```
  roslaunch vehicle_core nav_visual.launch
  ```
  
4) Enable the vehicle pilot (for safety the pilot is not sending thruster commands if not enabled by the user):
  ```
  rosservice call /pilot/switch "request: true"
  ```
  
5) Send a command to the pilot using the command-line (i.e. move the vehile to zero position):
  ```
  rostopic pub /pilot/position_req vehicle_interface/PilotRequest "position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
  ```

Extra: For trajectory tracking, enable the path:
  ```
  roslaunch vehicle_core path_controller.launch
  vehicle_core/tests/pp_lines_path.sh
  vehicle_core/scripts/path_cli.sh start
  ```
  
Extra: Use a different control law, run this instead of `pilot_sim.launch`:
  ```
  roslaunch vehicle_core auto_sim.launch
  ```
  
Run Real Operation
---
1) Start Nessie A (use SSH on Nessie5A):
  ```
  roslaunch vehicle_core nessie_A_basic.launch
  ```
  
2) Start Nessie B (use SSH on Nessie5B):
  ```
  roslaunch vehicle_core nessie_B_basic.launch
  ```
  
3) On Trieste (or any external laptop) start the joystick driver:
  ```
  roslaunch vehicle_core joystick_js0.launch
  ```
  
3b) Optional set the vehicle compass to use an artificial north (offset):
  ```
  rosparam set /conf/tcm/artificial_north 0.0
  ```
  
4) On Nessie A (after getting a correct initial position):
  ```
  roslaunch vehicle_core pilot_real.launch
  rosrun auv_nav auv_nav
  ```

6) Optional visualization (on Trieste or external laptop run):
  ```
  roslaunch vehicle_core nav_visual_nessie.launch
  ```
  
7) Enable the vehicle pilot (for safety the pilot is not sending thruster commands if not enabled by the user):
  ```
  rosservice call /pilot/switch "request: true"
  ```


Credits
-------

This work is part of **Nessie VIII AUV** project: *to boldly go where no robot has gone before!*


[semver]: http://semver.org/
[kiss]: http://en.wikipedia.org/wiki/KISS_principle
[tdd]: http://en.wikipedia.org/wiki/Test-driven_development
[solid]: http://en.wikipedia.org/wiki/SOLID_(object-oriented_design)
