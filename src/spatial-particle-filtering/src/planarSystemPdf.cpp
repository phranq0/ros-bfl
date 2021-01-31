// Definition of the probabilistic Motion Model (Unicycle)
// This model will be used from sampling the next pose (state) of each particle

#include "planarSystemPdf.h"
#include <wrappers/rng/rng.h>     // Random number generator

#define SYSMODEL_NUMCONDARGUMENTS_MOBILE 2        // Control vector size
#define SYSMODEL_DIMENSION_MOBILE 3               // State vector size                

namespace BFL
{
    using namespace MatrixWrapper;

    // Inherited constructor
    PlanarSystemPdf::PlanarSystemPdf(const Gaussian& additiveNoise)
        :   ConditionalPdf<ColumnVector,ColumnVector>(SYSMODEL_DIMENSION_MOBILE,SYSMODEL_NUMCONDARGUMENTS_MOBILE)
    {
        // Just stores noise as internal parameter
        _additiveNoise = additiveNoise;
    }

    // Destructor
    PlanarSystemPdf::~PlanarSystemPdf(){};

    // Sampling method
    bool PlanarSystemPdf::SampleFrom(Sample<ColumnVector>& one_sample, int method, void * args) const
    {
        ColumnVector state = ConditionalArgumentGet(0);
        ColumnVector vel = ConditionalArgumentGet(1);

        // System update
        state(1) += vel(1);
        state(2) += vel(2);
        state(3) += vel(3);

        // Sample from additive noise
        Sample<ColumnVector> noise;
        _additiveNoise.SampleFrom(noise, method, args);

        // State transition for one sample (particle)
        one_sample.ValueSet(state + noise.ValueGet());

        return true;
    }

}