// Definition of the conditional probability P(x(t)|x(t-1),u(t)) for 
// the planar agent entity
// The motion model here is linear 

#include "spatialagentconditionalgaussian.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng
                                 // libraries
#define NUMCONDARGUMENTS_PLANAR_AGENT 2

namespace BFL
{
  using namespace MatrixWrapper;


  SpatialAgentConditionalGaussian::SpatialAgentConditionalGaussian(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_PLANAR_AGENT)
  {
  }


  SpatialAgentConditionalGaussian::~SpatialAgentConditionalGaussian(){}

  ColumnVector SpatialAgentConditionalGaussian::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);
    state(1) += vel(1);
    state(2) += vel(2);
    state(3) += vel(3);
    state(4) += vel(4);
    state(5) += vel(5);
    state(6) += vel(6);
    return state + AdditiveNoiseMuGet();
  }

}//namespace BFL

