// This class represents a simulated planar agent, which behaves as an holonomic vehicle
// This implies the motion model to be linear instead of the sin/cos dependent classical unicycle model

#ifndef PLANAR_AGENT_HPP
#define PLANAR_AGENT_HPP


#include <model/analyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/gaussian.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/matrix/vector_wrapper.h>

#include "mobile_robot_wall_cts.h"
#include "planaragentconditionalgaussian.h"


namespace BFL{

/** The state of the planar agent is represented with a ColumnVector of three
* elements: the x and y position and the orientation. 
* The inputs of the robot are the rates of change of each component.
* Simulated measurements are handled directly in simulation loop
*/
     

  class PlanarAgent
    {
    public:
      // Constructor
      PlanarAgent();
      ~PlanarAgent();

      void Move(MatrixWrapper::ColumnVector inputs);
      const MatrixWrapper::ColumnVector Measure();
      const MatrixWrapper::ColumnVector GetState(); //method only for simulation purposes, collects the ground truth state

    private:
      Gaussian* _system_Uncertainty;
      PlanarAgentConditionalGaussian* _sys_pdf; 
      AnalyticSystemModelGaussianUncertainty* _sys_model;
      Gaussian* _measurement_Uncertainty;
      LinearAnalyticConditionalGaussian* _meas_pdf;
      LinearAnalyticMeasurementModelGaussianUncertainty* _meas_model;
      MatrixWrapper::ColumnVector _state;
    };
}

#endif