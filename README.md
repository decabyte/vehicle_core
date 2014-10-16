vehicle_core
============

This repo holds the ROS package of the Autonomous Vehicle Software (AVS) used within the OSL's vehicles. This package is meant to be python importable from other scripts and packages. It provides a consolidated set of modules, nodes, configuration and launch files to run basic missions with one of the OSL's vehicles. 

Guidelines
----------

Before attempting any modification to this repo please make sure thar: 
  - you are working on an up-to-date version of the `master` branch
  - the previous working version of the repo has been tagged (`git tag --list` to check the tags)
  - you have considered the option of creating a new branch for your feature (`git checkout -b <branch_name>` to create a new branch), after all this is the recommended approach!
  - you know what are you doing!
  
Initially this repo is providing packages using the rosbuild build system until the OSL vehicles are migrated to an upgraded version of the ROS. Later the rosbuild support is going to be dropped and the master branch will offer a catkinized package format.

Credits
-------

This work is part of **Nessie VIII AUV** project: *to boldly go where no robot has gone before!*

![nessie_logo](docs/nessie_tran.png)
