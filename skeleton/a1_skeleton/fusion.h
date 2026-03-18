#ifndef FUSION_H
#define FUSION_H

#include <vector>
#include "fusioninterface.h"
#include "rangerinterface.h"

class Fusion: public FusionInterface
{
public:
    
  Fusion(std::vector<RangerInterface*> rangers);


private:
  
  std::vector<std::vector<double>> data_; //!< This is to cater for getRawRangeData (which returns the raw data that was used for fusion))
  std::vector<RangerInterface*> rangers_; //!< A private copy of rangers @sa RangerInterface
  std::vector<pfms::Cell*> cells_; //!< A private copy of cells @sa Cell

};

#endif // FUSION_H
