// Definition of Measurement Model
// This model makes a direct confrontation on the state, computes the distance between measurement and (x,y) of each particle

#include "stateMeasurementPdf.h"
#include <wrappers/rng/rng.h>          // Random number generator
#include <cmath>

#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1       // Sensor array size
#define MEASMODEL_DIMENSION_MOBILE 3              // Measured state size

namespace BFL
{
    using namespace MatrixWrapper;

    stateMeasurementPdf::stateMeasurementPdf(const Gaussian& measNoise)
        : ConditionalPdf<ColumnVector,ColumnVector>(MEASMODEL_DIMENSION_MOBILE,MEASMODEL_NUMCONDARGUMENTS_MOBILE)
    {
        _measNoise = measNoise;
    }

    stateMeasurementPdf::~stateMeasurementPdf(){};

    // Get conditional probability, this is called for each particle
    Probability stateMeasurementPdf::ProbabilityGet(const ColumnVector& measurement) const
    {
        // Here z(k) is only conditioned by x(k), not u(k) like in more general model
        ColumnVector state = ConditionalArgumentGet(0);

        // Compute expected measurement for the current particle
        ColumnVector expected_measurement(3);
        ColumnVector distance(3);
        expected_measurement(1) = state(1);
        expected_measurement(2) = state(2);
        expected_measurement(3) = state(3);

        // Compute distance between simulated and real measurements
        distance = expected_measurement - measurement;
        
        // Computes the likelihood of the measurement by sampling
        // the probability of (em-m) in the noise distribution
        // The weight of the particle will be proportional to this particle
        return _measNoise.ProbabilityGet(distance);
    }
}