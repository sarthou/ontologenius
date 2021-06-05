#include <chrono>
#include <cstdlib>     /* srand, rand */
#include <ctime>       /* time */
#include <unordered_set>

#include <ros/ros.h>

#include "ontologenius/core/ontoGraphs/Ontology.h"
#include "ontologenius/core/reasoner/Reasoners.h"

using namespace std::chrono;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_tester");

  ontologenius::Ontology onto;
  ontologenius::Reasoners reasoners(&onto);
  reasoners.load();
  reasoners.list();

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_CRM.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_104_ChapterVII_Obstetrics.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_111_ChapterIII_Respiratory.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_113_ChapterX_Musculoskeletal.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_131_ChapterVIII_Oncology.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_136_ChapterI_Gastrointestinal.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_144_ChapterIX_Nutrition.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_161_ChapterXIII_Skin.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_164_ChapterVI_Endocrine.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_226_ChapterV_Infection.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_2383_VMP_A.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_297_ChapterIV_CNS.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_2_Chapters.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_346_Reinstated.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_38_ChapterXIV_Immunology.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_3914_VMP_B.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_409_Abstractions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_4319_Interactions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_474_ChapterII_Cardiovascular.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_476_BNFInteractions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_490_NewInteractions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_55_DrugsWithNoDissection.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_57_AdditionalInteractions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_635_Formulations.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_76_ChapterXV_Anaesthesia.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_8048_Components.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_84_ChapterXII_ENT.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DD_91_ChapterXI_Eye.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DissectionsDisease.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DissectionsDrugOntology.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DissectionsModel.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DissectionsSurgicalProcedure.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_1064_Sensory.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_118_Lymphoreticular.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_1507_Vascular.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_205_Endocrine.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_209_Respiratory.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_2157_Genitourinary.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_219_MinorSurgery.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_224_Therapeutic.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_307_Abstractions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_3227_Cardiothoracic.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_326_Ophthalmology.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_4076_Musculoskeletal.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_425_Paediatric.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_528_Neurosurgery.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_54_Breast.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_65_Demo.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_711_Orodental.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_755_SkinPlastic.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_85_Diagnostic.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_DS_935_Digestive.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_FoundationModel_ClinicalSituationModel.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_FoundationModel_DetailedMedicalCategorySpace.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_FoundationModel_GRAILSanctions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_FoundationModel_NamedDefinedEntities.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_FoundationModel.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_FoundationModel_TopClassesForMedicalDomain.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_FULL.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_GenericModel_GRAILSanctions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_GenericModel.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_GenericModel_PropertyChainAxioms.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_GenericModel_PropertyTypes.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_GenericModel_TopOntologyOfClasses.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Abstractions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_BasicMedicalScience.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Biochemistry.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Chemistry.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_ClinicalPragmatics.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Devices.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Drugs.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Genetics.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_HealthcareOrganisation.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_HumanAnatomy.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Investigations.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Microbiology.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Pathology.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Physiology.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_Psychiatry.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_SignsSymptoms.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_SocialAndEnvironment.owl");
  onto.readFromFile("/home/gsarthou/Downloads/OpenGALEN8/OWL_Sources/RDF/OpenGALEN8_MedicalExtensions_SurgicalProcedures.owl");

  high_resolution_clock::time_point t3 = high_resolution_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(t3 - t1);
  std::cout << "It took me " << time_span2.count() << " seconds to read" << std::endl;

  onto.close();

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(t2 - t3);
  std::cout << "It took me " << time_span.count() << " seconds to close" << std::endl;

  reasoners.runPostReasoners();

  high_resolution_clock::time_point t4 = high_resolution_clock::now();
  duration<double> time_span3 = duration_cast<duration<double>>(t4 - t2);
  std::cout << "It took me " << time_span3.count() << " seconds to runPostReasoners" << std::endl;

  std::vector<ontologenius::ClassBranch_t*> classes = onto.class_graph_.get();
  std::vector<ontologenius::Single_t<ontologenius::ClassBranch_t*>> res;
  ontologenius::ClassBranch_t* c_branch = nullptr;
  for(auto& c : classes)
  {
    c_branch = onto.class_graph_.findBranchUnsafe(c->value());
    res = c_branch->mothers_;
  }
    

  high_resolution_clock::time_point t5 = high_resolution_clock::now();
  duration<double> time_span4 = duration_cast<duration<double>>(t5 - t4);
  std::cout << "It took me " << time_span4.count() << " seconds to get " << classes.size() << " mothers" << std::endl;

  return 0;
}
