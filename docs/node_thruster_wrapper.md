# Vehicle Core: Thruster Wrapper

This node is used for injecting faults in the thruster subsystem. It works by intercepting the commands sent by
the pilot node to the thruster driver or the thruster simulator if used in simulation.

This node has its own associated launch file `thruster_wrapper.launch` that starts the pilot with a different output
topic `thrusters/request` instead of the standard `thrusters/commands`. Then it starts the `thruster_wrapper` node that
listens to the first topic and output the fault-injected commands on the standard one. 

This allows this system to work independently in simulations and with the real vehicle. The faults can be injected using
specific service calls exported by the `thruster_wrapper` node or by using the helper scripts provided.

## Usage
After starting the basic platform or the basic simulation environment, instead of running the default pilot launch file:

    roslaunch vehicle_core thruster_wrapper.launch
    
Then to inject a hard-limit fault into the system:

    roscd vehicle_core/scripts
    ./fault_clear.sh
    ./fault_inject.sh 51 85 85 85 85 85 # (fwd-port, fwd-std, lat-rear, lat-front, vert-rear, vert-front)
    
This limits the input commands of the port-side surge thruster at 60% of the nominal value. The `fault_inject` script
takes as input a number that is related to the lower-level commands that the pilot is sending to the thrusters. Therefore 
the following mapping between commands and percentage can be assumed:

    | Percentage | Value |
    |------------|-------|
    | 100        | 85    |
    | 80         | 68    |
    | 60         | 51    |
    | 40         | 34    |
    | 20         | 17    |
    | 0          | 0     |
    
This is because at the time of the introduction of this system the vehicle is using the Seabotix brushless DC thrusters
that needs to be limited in software to the maximum command `85` to limit their current usage within the specifications
reported in their datasheet. Do not attempt to push these thrusters above the 7A current limits otherwise their life will
be seriously shortened.
    
### Caution

Please think twice before using this on the real vehicle and never ever leave the vehicle unattended once a fault has 
been injected. This means please keep your hand on the joystick and the other one on the keyboard. Recruit a spare pair
of hands if necessary.
    
