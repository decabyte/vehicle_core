#!/bin/bash

rosservice call /path/control """
command: 'path'
points:
- values: [1.0, -1.8, 1.0, 0, 0, 0]
- values: [4.4, -9.8, 1.0, 0, 0, 0]
- values: [1.0, -9.8, 1.0, 0, 0, 0]
- values: [4.4, -1.8, 1.0, 0, 0, 0]
- values: [1.0, -1.8, 1.0, 0, 0, 0]
options:
- key: 'mode'
  value: 'fast'
- key: 'interpolation_method'
  value: 'linear'
- key: 'target_speed'
  value: '0.4'

- key: 'timeout'
  value: '300'
"""
