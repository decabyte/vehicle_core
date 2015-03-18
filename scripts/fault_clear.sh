#!/bin/bash

echo "Resetting Thruster Wrapper"
rosservice call /thrusters/faults """
- {key: 'fault_type', value: '0'}
"""
echo ""

exit 0
