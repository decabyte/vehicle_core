# Vehicle Core: Navigation Simulator

This node is used for simulating the behaviour of the AUV. It publishes the navigation and odometry topics that allow to use the vehicle software without deploying the vehicle. It also broadcasts the TF transformation needed to correctly display the vehicle in the RViz or UWSim interfaces.

## Topics

Here is a list of the topics related to this module:

### Subscribe

- /nav/sim/forces   (vehicle_interface/Vector6Stamped)
- /nav/sim/water    (vehicle_interface/FloatArrayStamped)

### Publish

- /nav/nav_sts      (auv_msgs/NavSts)
- /nav/odometry     (nav_msgs/Odometry)


## Water Current Simulation

This node provides the capability of simulating the presence of water currents in the navigation environment. Currents are injected in the vehicle model as an external disturbance with known velocity and orientation. The water current is assumed to be of known magnitude for the first 10 meters of the water layer then decay using a logarithm trend until the sea bottom. If not specified otherwise the `nav_sim` node assumes the sea bottom to be 1000 meters below the surface.

The effect of currents can be controlled using the `/nav/sim/water` topic and six parameters:
    
- Mean value of water current surface speed (`mu_v`, m/s)
- Variance value of water current surface speed (`sigma_v`, (m/s)^2)
- Mean value of water current angle of attack in azimuth (`mu_b`, radians, horizontal plane)
- Variance value of angle of attack in azimuth (`sigma_b`, radians^2)
- Mean value of water current angle of attack in elevation (`mu_a`, radians, vertical plane)
- Variance value of angle of attack in elevation (`sigma_a`, radians^2)

For instance using the command line tool:

    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.1, 0.001, 0.0, 0.001, 0.0, 0.001]"
    
This will tell the `nav_sim` node to simulate a water current with surface speed of 0.1 m/s and orientation north to south, using a normal model (mu, sigma) for the parameters that define the water current at the surface.
    
