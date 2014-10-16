#!/bin/bash
 rosservice call /path/control """header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
command: 'reset'
"""

