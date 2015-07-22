#!/bin/bash

rostopic pub -1 /path/request vehicle_interface/PathRequest """
command: 'path'
points:
- values: [  4.   , -10.   ,   1.5   ,   0.   ,   0.   ,   0.   ]
- values: [  3.778,  -9.111,   1.4   ,   0.   ,   0.   ,   0.   ]
- values: [  3.556,  -8.222,   1.3   ,   0.   ,   0.   ,   0.   ]
- values: [  3.333,  -7.333,   1.2   ,   0.   ,   0.   ,   0.   ]
- values: [  3.111,  -6.444,   1.1   ,   0.   ,   0.   ,   0.   ]
- values: [  2.889,  -5.556,   1.0   ,   0.   ,   0.   ,   0.   ]
- values: [  2.667,  -4.667,   0.9   ,   0.   ,   0.   ,   0.   ]
- values: [  2.444,  -3.778,   0.8   ,   0.   ,   0.   ,   0.   ]
- values: [  2.222,  -2.889,   0.7   ,   0.   ,   0.   ,   0.   ]
- values: [  2.   ,  -2.   ,   0.6   ,   0.   ,   0.   ,   0.   ]
options:
- {key: 'mode', value: 'simple'}
- {key: 'target_speed', value: '0.5'}
- {key: 'timeout', value: '300'}
""" 
