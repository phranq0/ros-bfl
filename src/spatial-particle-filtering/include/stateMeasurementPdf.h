// Header file which defines nonlinear probability density function (aka Measurement Model)
// related to a point-shaped mobile robot, which is a conditional probability density defined as P(z(k)|x(k))
// This model makes a direct confrontation on the state, computes the distance between measurement and (x,y) of each particle

#ifndef __STATE_MEAS_MOBILE__
#define __STATE_MEAS_MOBILE__

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>

namespace BFL
{
    // Non linear conditional gaussian, inherited from general conditional pdf
    class stateMeasurementPdf : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
        {
            public:
                // Constructor
                stateMeasurementPdf(const Gaussian& measNoise);

                // Destructor 
                virtual ~stateMeasurementPdf();

                // This gets the measurement model, the P(z(k)|x(k))
                virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

            private:
                Gaussian _measNoise;

        };
}

#endif