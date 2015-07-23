#!/bin/bash

rostopic pub -1 /path/request vehicle_interface/PathRequest """
command: 'path'
points:
- values: [2.0, -2.0, 1.0, 0, 0, 0]
- values: [4.0, -2.0, 1.0, 0, 0, 0]
- values: [4.0, -6.0, 1.0, 0, 0, 0]
- values: [2.0, -6.0, 1.0, 0, 0, 0]
- values: [2.0, -10.0, 1.0, 0, 0, 0]
- values: [4.0, -10.0, 1.0, 0, 0, 0]
- values: [2.0, -2.0, 1.0, 0, 0, 0]

options:
- {key: 'mode', value: 'fast'}
- {key: 'target_speed', value: '0.5'}
- {key: 'timeout', value: '300'}
""" 
