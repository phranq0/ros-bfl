// Parameters file for the spatial agent simulation
// Contains all the constant which descrive uncertainty and models

#ifndef __SPATIAL_AGENT_PARAMS_
#define __SPATIAL_AGENT_PARAMS_

#include <cmath>

// To use measurements specify 1
// To do deadreckoning (no measurements) specify 0
#define USE_MEASUREMENTS 1

#define DELTA_T 1	        // Delta t (for discretisation)
#define NUM_TIME_STEPS 101  // Number of steps that  simulation is running

// Sizes
#define STATE_SIZE 6 //state: x,y,z,alpha,beta,gamma
#define INPUT_SIZE 6 //input: vx,vy,vz,wx,wy,wz
#define MEAS_SIZE 3  //measurment: distance from a point in space

//Initial position and orientation of the mobile robot
#define X_0 0
#define Y_0 0
#define Z_0 0
#define ALPHA_0  0
#define BETA_0  0
#define GAMMA_0  0

#define NUM_SAMPLES 500 // Default Number of Samples (Particles)
#define RESAMPLE_PERIOD 0 // Default Resample Period
#define RESAMPLE_THRESHOLD (NUM_SAMPLES/4.0) // Threshold for Dynamic Resampling

// ------------- Prior Estimate Parameters
// Initial estimate of position and orientation
#define PRIOR_MU_X 0
#define PRIOR_MU_Y 0
#define PRIOR_MU_Z 0
#define PRIOR_MU_ALPHA M_PI/4	
#define PRIOR_MU_BETA M_PI/4	
#define PRIOR_MU_GAMMA M_PI/4

// Initial covariances of position and orientation
#define PRIOR_COV_X pow(0.6,2)
#define PRIOR_COV_Y pow(0.6,2)
#define PRIOR_COV_Z pow(0.3,2)
#define PRIOR_COV_ALPHA pow(M_PI/8,2)
#define PRIOR_COV_BETA pow(M_PI/8,2)
#define PRIOR_COV_GAMMA pow(M_PI/8,2)

// System Noise
#define MU_SYSTEM_NOISE_X 0.0 
#define MU_SYSTEM_NOISE_Y 0.0 
#define MU_SYSTEM_NOISE_Z 0.0
#define MU_SYSTEM_NOISE_ALPHA 0.0
#define MU_SYSTEM_NOISE_BETA 0.0
#define MU_SYSTEM_NOISE_GAMMA 0.0
#define SIGMA_SYSTEM_NOISE_X pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Y pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Z pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_ALPHA pow(2*M_PI/180,2)
#define SIGMA_SYSTEM_NOISE_BETA pow(2*M_PI/180,2)
#define SIGMA_SYSTEM_NOISE_GAMMA pow(2*M_PI/180,2)

// System Noise 
#define MU_SYSTEM_NOISE_X_ROB 0.0 
#define MU_SYSTEM_NOISE_Y_ROB 0.0 
#define MU_SYSTEM_NOISE_Z_ROB 0.0
#define MU_SYSTEM_NOISE_ALPHA_ROB 0.0
#define MU_SYSTEM_NOISE_BETA_ROB 0.0
#define MU_SYSTEM_NOISE_GAMMA_ROB 0.0
#define SIGMA_SYSTEM_NOISE_X_ROB pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Y_ROB pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Z_ROB pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_ALPHA_ROB pow(0.2*M_PI/180,2)
#define SIGMA_SYSTEM_NOISE_BETA_ROB pow(0.2*M_PI/180,2)
#define SIGMA_SYSTEM_NOISE_GAMMA_ROB pow(0.2*M_PI/180,2)

// System Noise for simulation
#define SIM_FACTOR 1000 //The system covariance in simulation is SIM_FACTOR
                        //smaller than the system covariance of the systemmodel
// Actually the system covariance is neglected in the simulated robot,
// in order to have ground truth, the uncertainty is taken into account
// in the noisy measurements

// Measurement noise
#define SIGMA_MEAS_NOISE pow(0.5,2)
#define MU_MEAS_NOISE 0.0

#endif 