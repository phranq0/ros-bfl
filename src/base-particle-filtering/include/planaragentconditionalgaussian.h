// This class represents the conditional probability P(x(t)|x(t-1),u(t)) for 
// the planar agent entity
// The motion model here is linear 

#ifndef __PLANAR_AGENT_CONDITIONAL_GAUSSIAN__
#define __PLANAR_AGENT_CONDITIONAL_GAUSSIAN__

#include <pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{
  
  class PlanarAgentConditionalGaussian : public AnalyticConditionalGaussianAdditiveNoise
    {
    public:
      /// Constructor
      /** @pre:  Every Matrix should have the same amount of rows!
	  This is currently not checked.  The same goes for the number
	  of columns, which should be equal to the number of rows of
	  the corresponding conditional argument!
	  @param ratio: vector containing the different matrices of
	  the linear relationship between the conditional arguments
	  and \f$\mu\f$
	  @param additiveNoise Pdf representing the additive Gaussian uncertainty
      */
      PlanarAgentConditionalGaussian( const Gaussian& additiveNoise);

      /// Destructor
      virtual ~PlanarAgentConditionalGaussian();

      // redefine virtual functions
      // Unlike the mobile robot, here no Jacobian is needed
      virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
    };

} // End namespace BFL

#endif //

