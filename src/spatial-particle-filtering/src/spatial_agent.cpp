// Definition of the class which describes holonomic planar agent
// Many aspects are taken from the 'mobile_robot.cpp' definition 

#include "spatial_agent.h"
#include "spatialagentconditionalgaussian.h"

using namespace MatrixWrapper;

namespace BFL
{

    SpatialAgent::SpatialAgent():
        _state(STATE_SIZE)
      {

	      // Initializing state vector
	      _state(1) = X_0;
	      _state(2) = Y_0;
	      _state(3) = Z_0;
        _state(4) = ALPHA_0;
	      _state(5) = BETA_0;
	      _state(6) = GAMMA_0;

	      // Sys noise (here not needed, left just as placeholder, since I'm simulating an exact path)
	      ColumnVector sys_noise_Mu(STATE_SIZE);
        for (int i=0; i < STATE_SIZE; i++)
        {
            sys_noise_Mu(i+1) = 0;
        }
	      
	      SymmetricMatrix sys_noise_Cov(STATE_SIZE);
        sys_noise_Cov = 0.0;
        //sys_noise_Cov(1,1) = 0;
	      //sys_noise_Cov(1,2) = 0.0;
	      //sys_noise_Cov(1,3) = 0.0;
	      //sys_noise_Cov(2,1) = 0.0;
        //sys_noise_Cov(2,2) = 0;
	      //sys_noise_Cov(2,3) = 0.0;
	      //sys_noise_Cov(3,1) = 0.0;
	      //sys_noise_Cov(3,2) = 0.0;
        //sys_noise_Cov(3,3) = 0;
        _system_Uncertainty = new Gaussian(sys_noise_Mu, sys_noise_Cov);

        // Create the model, still defined as probabilistic for convenience, but 
        // covariance is null since ground truth pose is needed
        _sys_pdf = new SpatialAgentConditionalGaussian(*_system_Uncertainty);
        _sys_model = new AnalyticSystemModelGaussianUncertainty(_sys_pdf);

	      // Measurements simulation (actually unused, are taken directly
        // from the simulation loop)
	      SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
	      meas_noise_Cov(1,1) = 0.005;
        meas_noise_Cov(2,2) = 0.005;
	      ColumnVector meas_noise_Mu(MEAS_SIZE);
	      meas_noise_Mu(1) = MU_MEAS_NOISE;
        meas_noise_Mu(2) = MU_MEAS_NOISE;
	      _measurement_Uncertainty = new Gaussian(meas_noise_Mu, meas_noise_Cov);

        // create matrix _meas_model for linear measurement model
        // This just emulates a gps-like reading
	      Matrix H(MEAS_SIZE,STATE_SIZE);
        H = 0.0;
        H(1,1) = 1;
        H(1,2) = 1;
        H(1,3) = 0;

        // create the measurement model
        _meas_pdf = new LinearAnalyticConditionalGaussian(H, *_measurement_Uncertainty);
        _meas_model = new LinearAnalyticMeasurementModelGaussianUncertainty(_meas_pdf);
        }

          SpatialAgent::~SpatialAgent()
            {
              delete _system_Uncertainty;
              delete _sys_pdf;
              delete _sys_model;
              delete _measurement_Uncertainty;
              delete _meas_pdf;
              delete _meas_model;
            }

          void
          SpatialAgent::Move(ColumnVector inputs)
          {
            _state = _sys_model->Simulate(_state,inputs);
          }


          const ColumnVector
          SpatialAgent::Measure()
          {
            return _meas_model->Simulate(_state);
          }


          const ColumnVector
          SpatialAgent::GetState()
          {
            return _state;
          }
}     