#!/bin/bash

rostopic pub -1 /path/request vehicle_interface/PathRequest """
command: 'path'
points:
- values: [3.0, -6.0, 0.2, 0, 0, 0]
options:
- {key: 'mode', value: 'fast'}
- {key: 'target_speed', value: '0.5'}
- {key: 'timeout', value: '300'}
""" 
