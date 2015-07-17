# Vehicle Core: Navigation Simulator

This node is used for simulating the behaviour of the AUV. It publishes the navigation and odometry topics that allow to use the vehicle software without deploying the vehicle. It also broadcasts the TF transformation needed to correctly display the vehicle in the RViz or UWSim interfaces.

## Topics

Here is a list of the topics related to this module:

### Subscribe

- /nav/sim/forces       (vehicle_interface/Vector6Stamped)
- /nav/sim/water        (vehicle_interface/FloatArrayStamped)

### Publish

- /nav/nav_sts          (auv_msgs/NavSts)
- /nav/odometry         (nav_msgs/Odometry)
- /nav/sim/currents     (vehicle_interface/FloatArrayStamped)


## Water Current Simulation

This node provides the capability of simulating the presence of water currents in the navigation environment. Currents are injected in the vehicle model as an external disturbance with known velocity and orientation. The water current is assumed to be of known magnitude for the first 10 meters of the water layer then decay using a logarithm trend until the sea bottom. If not specified otherwise the `nav_sim` node assumes the sea bottom to be 1000 meters below the surface.

The effect of currents can be controlled using the `/nav/sim/water` topic and six parameters:
    
- Maximum value of water current surface speed (`v`, m/s)
- Variance value of water current surface speed (`sigma_v`, (m/s)^2)
- First order Gauss-Markov process coefficient (`mu`, [0.0, 0.1])

- Mean value of water current angle of attack in azimuth (`b`, radians, horizontal plane)
- Variance value of angle of attack in azimuth (`sigma_b`, radians^2)
- Mean value of water current angle of attack in elevation (`a`, radians, vertical plane)
- Variance value of angle of attack in elevation (`sigma_a`, radians^2)

For instance using the command line tool:
    
    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.25, 0.05, 0.01]"
    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.25, 0.05, 0.01, 0.0, 0.001, 0.0, 0.001]"
    
This will tell the `nav_sim` node to simulate a water current with surface speed of 0.1 m/s and orientation north to south, using a first order gauss-markov process (defined by the coefficients `mu`, `v`, `sigma_v`). The `v` term specifies the maximum allowed current speed (bounded between 0.0 and `v`), the `sigma_v` term specifies the amplitude of the gaussian noise term in the process and the `mu` term regulates the first-order dependency of the whole GM process.
    
To disable the effect of currents:

    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.0, 0.001, 0.0]"
    
This will set to zero the maximum allowed current speed and it will disable the GM process. Together with this the navigation simulator is publish using the topic `nav/sim/currents` the actual state of the current simulator (current speed, angle b, angle a). This can later be used if recorded during the experiments.
