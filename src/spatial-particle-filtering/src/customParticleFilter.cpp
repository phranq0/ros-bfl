// Class which extends the basic Particle Filter class implemented in Bfl
// This just simply adds a 'get method' for reading samples, which otherwise are private
// Apart from this is identical to the BootstrapFilter

#include "customParticleFilter.h"

using namespace MatrixWrapper;
using namespace BFL;

CustomParticleFilter::CustomParticleFilter(MCPdf<ColumnVector> *prior, int resampleperiod, double resamplethreshold, int resamplescheme):
      BootstrapFilter<ColumnVector,ColumnVector>(prior, resampleperiod, resamplethreshold, resamplescheme)
{

  }

  vector<WeightedSample<ColumnVector> > CustomParticleFilter::getNewSamples()
  {
      return _new_samples;
  }
