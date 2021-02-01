// Definition of the probabilistic Motion Model 
// This model will be used from sampling the next pose (state) of each particle

#include "spatialSystemPdf.h"
#include "spatial_agent_params.h"
#include <wrappers/rng/rng.h>     // Random number generator

#define SYSMODEL_NUMCONDARGUMENTS_MOBILE 2        // Number of conditional parameters
#define SYSMODEL_DIMENSION_MOBILE 3               // State vector size                

namespace BFL
{
    using namespace MatrixWrapper;

    // Inherited constructor
    SpatialSystemPdf::SpatialSystemPdf(const Gaussian& additiveNoise)
        :   ConditionalPdf<ColumnVector,ColumnVector>(SYSMODEL_DIMENSION_MOBILE,SYSMODEL_NUMCONDARGUMENTS_MOBILE)
    {
        // Just stores noise as internal parameter
        _additiveNoise = additiveNoise;
    }

    // Destructor
    SpatialSystemPdf::~SpatialSystemPdf(){};

    // Sampling method
    bool SpatialSystemPdf::SampleFrom(Sample<ColumnVector>& one_sample, int method, void * args) const
    {
        ColumnVector state = ConditionalArgumentGet(0);
        ColumnVector vel = ConditionalArgumentGet(1);

        // System update
        for (int i = 0; i < STATE_SIZE; i++)
        {
            state(i+1) += vel(i+1);
        }

        // Sample from additive noise
        Sample<ColumnVector> noise;
        _additiveNoise.SampleFrom(noise, method, args);

        // State transition for one sample (particle)
        one_sample.ValueSet(state + noise.ValueGet());

        return true;
    }

}