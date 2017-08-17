#include "gait_generator.h"

namespace spyndra
{
class CsvGaitGenerator : GaitGenerator
{
public:
  CsvGaitGenerator(){}
  CsvGaitGenerator(const CsvGaitGenerator& rhs){}
  ~CsvGaitGenerator(){}

  void foo();
};

//void CsvGaitGenerator::foo(){}
}
