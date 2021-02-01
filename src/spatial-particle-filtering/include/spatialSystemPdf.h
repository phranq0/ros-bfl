// Declaration of Motion model for linear spatial model
// This is the conditional probability density function which 
// relates the actual state to the next state

#ifndef __LINEAR_SPATIAL_SYSTEM__
#define __LINEAR_SPATIAL_SYSTEM__

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>

namespace BFL
{
    // Nonlinear conditional gaussian pdf
    class SpatialSystemPdf : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
        {
            public:
                // Constructor
                SpatialSystemPdf( const Gaussian& additiveNoise);

                // Destructor
                virtual ~SpatialSystemPdf();

                // Method for sampling from the stochastic model
                virtual bool SampleFrom ( Sample<MatrixWrapper::ColumnVector>& one_sample, int method=DEFAULT, void * args=NULL) const;

            private:
                Gaussian _additiveNoise;
        };
}

#endif