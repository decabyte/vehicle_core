#!/bin/bash

# read command line arguments and use "100" as default
#   the minus sign in the bash expansion is not the mathematical minus sign
TH0=${1:-100}
TH1=${2:-100}
TH2=${3:-100}
TH3=${4:-100}
TH4=${5:-100}
TH5=${6:-100}

# script body
echo "Resetting Thruster Wrapper"
rosservice call /thrusters/faults """
- {key: 'fault_type', value: '0'}
"""
echo ""

echo "Injecting fault"
rosservice call /thrusters/faults """
- {key: 'fault_type', value: 'hard_limit'}
- {key: 'th_min', value: '-$TH0, -$TH1, -$TH2, -$TH3, -$TH4, -$TH5'}
- {key: 'th_max', value: '$TH0, $TH1, $TH2, $TH3, $TH4, $TH5'}
"""
echo ""

exit 0
