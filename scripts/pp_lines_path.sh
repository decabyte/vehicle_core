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
  value: 'lines'
- key: 'timeout'
  value: '300'
"""

#- values: [1,-1,1,0,0,-1.047]
#- values: [5.4,-9.8,1,0,0,-1.047]