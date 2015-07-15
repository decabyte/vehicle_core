#!/bin/bash

rostopic pub -1 /path/request vehicle_interface/PathRequest """
command: 'path'
points:
- values: [  6.   , -10.   ,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  6.   , -10.   ,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  5.556,  -9.111,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  5.111,  -8.222,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  4.667,  -7.333,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  4.222,  -6.444,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  3.778,  -5.556,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  3.333,  -4.667,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  2.889,  -3.778,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  2.444,  -2.889,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  2.   ,  -2.   ,   1.   ,   0.   ,   0.   ,   0.   ]
- values: [  2.   ,  -2.   ,   1.   ,   0.   ,   0.   ,   0.   ]

options:
- {key: 'mode', value: 'simple'}
- {key: 'target_speed', value: '0.5'}
- {key: 'timeout', value: '300'}
""" 
