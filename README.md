# ros-bfl
ROS workspace for testing and experimenting Bayesian filtering algorithms (Particle filters, Kalman filters)
Requires an up-and-running version of the Orocos Bayesian Filtering Library (BFL), better if installed within ROS (valid only for Kinetic and Melodic)

The examples are built using a kinematic point-shaped robot included in bfl

Up to now only basic Bootstrap Particle Filter simulation is implemented

The 'simulation' branch runs the filter on a simulated entity, up to now the implemented agents are the following:
- Non-holonomic planar vehicle
- Holonomic planar agent
- Holonomic 3D agent (for 6D pose estimation)

The 'master' branch runs the filter within the ROS framework; the filter updates (predict and correct) are triggered
by input model and sensor model callbacks
