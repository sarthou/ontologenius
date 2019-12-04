#include "ontoloGenius/core/reasoner/ConfigReader.h"

int main()
{
  ontologenius::ConfigReader reader;
  reader.read("/home/gsarthou/Robots/Pr2/Sherlock/catkin_ws/src/hri_rs_package/descriptors/analysis_engines/hri_pipeline.yaml");
  reader.display();
  return 0;
}
