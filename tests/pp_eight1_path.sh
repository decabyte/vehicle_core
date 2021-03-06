#!/bin/bash

rostopic pub -1 /path/request vehicle_interface/PathRequest """
command: 'path'
points:
- values: [2.0, -2.0, 1.0, 0, 0, 0]
- values: [4.0, -10.0, 1.0, 0, 0, 0]
- values: [2.0, -10.0, 1.0, 0, 0, 3.14]
- values: [4.0, -2.0, 1.0, 0, 0, 0]
- values: [2.0, -2.0, 1.0, 0, 0, 3.14]

options:
- key: 'mode'
  value: 'lines'
- key: 'timeout'
  value: '300'
"""
