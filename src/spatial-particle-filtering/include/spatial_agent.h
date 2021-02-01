// This class represents a simulated 3D spatial agent, which behaves as an holonomic point
// Motion model is linear, its pose is described by a 6D state vector
#ifndef SPATIAL_AGENT_HPP
#define SPATIAL_AGENT_HPP


#include <model/analyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/gaussian.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/matrix/vector_wrapper.h>

#include "spatial_agent_params.h"
#include "spatialagentconditionalgaussian.h"


namespace BFL{

/** The state of the spatial agent is represented with a ColumnVector of six
* elements: the x,y,z position and orientation about same axes (describes in euler angles) 
* The inputs of the robot are the rates of change of each component.
* Simulated measurements are handled directly in simulation loop
*/
     
  class SpatialAgent
    {
    public:
      // Constructor
      SpatialAgent();
      ~SpatialAgent();

      void Move(MatrixWrapper::ColumnVector inputs);
      const MatrixWrapper::ColumnVector Measure();
      const MatrixWrapper::ColumnVector GetState(); //method only for simulation purposes, collects the ground truth state

    private:
      Gaussian* _system_Uncertainty;
      SpatialAgentConditionalGaussian* _sys_pdf; 
      AnalyticSystemModelGaussianUncertainty* _sys_model;
      Gaussian* _measurement_Uncertainty;
      LinearAnalyticConditionalGaussian* _meas_pdf;
      LinearAnalyticMeasurementModelGaussianUncertainty* _meas_model;
      MatrixWrapper::ColumnVector _state;
    };
}

#endif