// Definition of the conditional probability P(x(t)|x(t-1),u(t)) for 
// the planar agent entity
// The motion model here is linear 

#include "planaragentconditionalgaussian.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng
                                 // libraries
#define NUMCONDARGUMENTS_PLANAR_AGENT 2

namespace BFL
{
  using namespace MatrixWrapper;


  PlanarAgentConditionalGaussian::PlanarAgentConditionalGaussian(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_PLANAR_AGENT)
  {
  }


  PlanarAgentConditionalGaussian::~PlanarAgentConditionalGaussian(){}

  ColumnVector PlanarAgentConditionalGaussian::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);
    state(1) += vel(1);
    state(2) += vel(2);
    state(3) += vel(3);
    return state + AdditiveNoiseMuGet();
  }

}//namespace BFL

