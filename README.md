# Vehicle Core

This repo holds the ROS package of the Autonomous Vehicle Software (AVS) used with OSL vehicles. This package is
meant to be python importable from other scripts and packages. It provides a consolidated set of modules, nodes,
configuration and launch files to run basic missions with one of the OSL's vehicles.

Detailed documentation can be found in the [docs](docs) folder of this repo.

## Requirements
  - A valid ROS installation (version Hydro, Indigo or later is suggested)
  - ROS joy package from ROS Joystick Drivers (`$ sudo apt-get install ros-<VERSION>-joy`)
  - Python 2.7+
  - Numpy 1.8+
  - Scipy 0.13+

## Run Simulator

1) Run vehicle simulator (this includes the navigation simulator, the vehicle pilot with simulation config and the path controller):

    roslaunch vehicle_core simulator.launch use_gui:=true

  
2) Enable the vehicle pilot (for safety the pilot is not sending thruster commands if not enabled by the user):

    rosservice call /pilot/switch "request: true"
  
3) Send a command to the pilot using the command-line (i.e. move the vehile to zero position):

    rostopic pub /pilot/position_req vehicle_interface/PilotRequest "position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    
### Customization

To use a different control law, run:

    roslaunch vehicle_core simulator.launch use_gui:=true ctrl_config:=pid_sim_model.yaml

where `pid_sim_model.yaml` is one of the controller configuration files you can find in the `conf` directory.

To the path controller please use any of the path scripts included in this package:

    vehicle_core/tests/pp_lines_path.sh
    vehicle_core/scripts/path_cli.sh start

  
Behind the scenes the simulation.launch scripts is running the following nodes:
 
1) Run the navigation simulator

    roslaunch vehicle_core nav_sim.launch

2) Run vehicle controller

    roslaunch vehicle_core pilot_sim.launch

3) Run visualization

    roslaunch vehicle_core nav_visual.launch

4) Run the path controller:

    roslaunch vehicle_core path_controller.launch

  
## Run Real Operation
1) Start Nessie A (use SSH on NessieA):

    roslaunch vehicle_core nessie_A_basic.launch
  
2) Start Nessie B (use SSH on NessieB):

    roslaunch vehicle_core nessie_B_basic.launch
  
3) On Trieste (or any external laptop) start the joystick driver (see Using Joystick for more information):

    roslaunch vehicle_core joystick.launch
  
3b) Optional set the vehicle compass to use an artificial north (offset):

    rosnode kill /Compass
    rosparam set /conf/tcm/artificial_north 0.0
    rosrun pni_compass compass
  
4) On Nessie A (after getting a correct initial position):

    roslaunch vehicle_core pilot.launch ctrl_config:=pid_real.yaml
    rosrun auv_nav nav

5) Optional visualization (on Trieste or external laptop run):
    
    roslaunch vehicle_core nav_visual.launch
  
6) Enable the vehicle pilot (for safety the pilot is not sending thruster commands if not enabled by the user):

    rosservice call /pilot/switch "request: true"
    
7) For a desired path to be activated:

    roslaunch vehicle_core path_controller.launch
    vehicle_core/tests/pp_lines_path.sh
    vehicle_core/scripts/path_cli.sh start
  

## Using Joystick
1) Connect the joystick to the computer you are using. At this point your computer will assign a path to it, typically
 it will be:
    
    /dev/input/js0
    
 However, if other devices are connected this can be different.
   
2) Make sure the computer you are running is connecting to roscore on Nessie A. On trieste there are aliases for 
 switching this (look at .bashrc).
 
3) Launch file which corresponds to the joystick. If the joystick you are intending to use is not at /dev/input/js0 
 then you will have to specify its path as an argument, e.g.:
 
    roslaunch vehicle_core joystick.launch dev:=/dev/input/js2
    
 Otherwise you do not have to specify it explicitly.
 
4) Before using the joystick consider these:
  - after some time of being inactive the joystick will go to sleep, press D-pad button (above the analogue sticks) to 
  make sure it is awake
  - if moving left analogue stick has no effect on the vehicle press D-pad button (toggles activation of D-pad/left
  analogue)
  - have spare batteries close by in case the current ones run out; losing control over the vehicle in key moment can 
  lead to damaging the vehicle
  
5) The following is the joystick mapping:
  - left analogue stick applies forces in surge and sway
  - right analogue stick applies yaw force by default
  - while R2 is pressed right analogue applies force in vertical directions
  - X button toggles 'autopilot' - vehicle goes to the last point requested
  - square button sends current position request
  - (i.e. sequential combination square + X is 'hold this position')
  - circle button switches off the thrusters at the driver level (use to make sure that the vehicle cannot move)
  - R1 button is boost: the forces due to other actions are increased while the button is pressed
  
## Guidelines

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

## Credits

This work is part of **Nessie VIII AUV** project: *to boldly go where no robot has gone before!*


[semver]: http://semver.org/
[kiss]: http://en.wikipedia.org/wiki/KISS_principle
[tdd]: http://en.wikipedia.org/wiki/Test-driven_development
[solid]: http://en.wikipedia.org/wiki/SOLID_(object-oriented_design)
