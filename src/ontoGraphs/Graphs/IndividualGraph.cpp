#include "ontoloGenius/ontoGraphs/Graphs/IndividualGraph.h"

IndividualGraph::IndividualGraph(ClassGraph* classes, PropertyGraph* properties)
{
  classes_ = classes;
  properties_ = properties;
}

IndividualGraph::~IndividualGraph()
{
  for(unsigned int i = 0; i < individuals_.size(); i++)
    delete individuals_[i];

  individuals_.clear();
}

void IndividualGraph::close()
{
  container_.load(individuals_);
}

void IndividualGraph::add(std::string value, IndividualVectors_t& individual_vector)
{
  //am I created ?
  IndividualBranch_t* me = nullptr;
  for(unsigned int i = 0; i < individuals_.size(); i++)
  {
    if(individuals_[i]->value_ == value)
    {
      me = individuals_[i];
      individuals_.erase(individuals_.begin() + i);
      break;
    }
  }

  if(me == nullptr)
    me = new IndividualBranch_t(value);

  /**********************
  ** Class assertion
  **********************/
  //for all my classes
  for(unsigned int is_a_i = 0; is_a_i < individual_vector.is_a_.size(); is_a_i++)
  {
    bool i_find_my_is_a_ = false;

    //is a root my class ?
    for(unsigned int root_i = 0; root_i < classes_->roots_.size(); root_i++)
      if(individual_vector.is_a_[is_a_i] == classes_->roots_[root_i]->value_)
      {
        me->is_a_.push_back(classes_->roots_[root_i]);
        i_find_my_is_a_ = true;
      }

    //is a branch my class ?
    for(unsigned int branch_i = 0; branch_i < classes_->branchs_.size(); branch_i++)
      if(individual_vector.is_a_[is_a_i] == classes_->branchs_[branch_i]->value_)
      {
        me->is_a_.push_back(classes_->branchs_[branch_i]);
        i_find_my_is_a_ = true;
      }

    //is a tmp mother is my class ?
    for(unsigned int branch_i  =0; branch_i < classes_->tmp_mothers_.size(); branch_i++)
      if(individual_vector.is_a_[is_a_i] == classes_->tmp_mothers_[branch_i]->value_)
      {
        me->is_a_.push_back(classes_->tmp_mothers_[branch_i]);
        i_find_my_is_a_ = true;
      }

    //I create my class
    if(!i_find_my_is_a_)
    {
      ObjectVectors_t empty_vectors;
      classes_->add(individual_vector.is_a_[is_a_i], empty_vectors);
      for(unsigned int root_i = 0; root_i < classes_->roots_.size(); root_i++)
        if(individual_vector.is_a_[is_a_i] == classes_->roots_[root_i]->value_)
        {
          me->is_a_.push_back(classes_->roots_[root_i]);
          i_find_my_is_a_ = true;
        }
    }
  }

  /**********************
  ** Object Property assertion name
  **********************/
  //for all my properties
  for(unsigned int property_i = 0; property_i < individual_vector.properties_name_.size(); property_i++)
  {
    bool i_find_my_properties = false;

    //is a root my properties ?
    for(unsigned int root_i = 0; root_i < properties_->roots_.size(); root_i++)
      if(individual_vector.properties_name_[property_i] == properties_->roots_[root_i]->value_)
      {
        me->properties_name_.push_back(properties_->roots_[root_i]);
        i_find_my_properties = true;
      }

    //is a branch my properties ?
    for(unsigned int branch_i = 0; branch_i < properties_->branchs_.size(); branch_i++)
      if(individual_vector.properties_name_[property_i] == properties_->branchs_[branch_i]->value_)
      {
        me->properties_name_.push_back(properties_->branchs_[branch_i]);
        i_find_my_properties = true;
      }

    //is a tmp mother is my properties ?
    for(unsigned int branch_i  =0; branch_i < properties_->tmp_mothers_.size(); branch_i++)
      if(individual_vector.properties_name_[property_i] == properties_->tmp_mothers_[branch_i]->value_)
      {
        me->properties_name_.push_back(properties_->tmp_mothers_[branch_i]);
        i_find_my_properties = true;
      }

    //I create my properties
    if(!i_find_my_properties)
    {
      PropertyVectors_t empty_vectors;
      properties_->add(individual_vector.properties_name_[property_i], empty_vectors);
      for(unsigned int root_i = 0; root_i < properties_->roots_.size(); root_i++)
        if(individual_vector.properties_name_[property_i] == properties_->roots_[root_i]->value_)
        {
          me->properties_name_.push_back(properties_->roots_[root_i]);
          i_find_my_properties = true;
        }
    }
  }

  /**********************
  ** Object Property assertion on class
  **********************/
  //for all my classes
  for(unsigned int is_a_i = 0; is_a_i < individual_vector.properties_on_.size(); is_a_i++)
  {
    bool i_find_my_properties_on = false;

    //is a root my class ?
    for(unsigned int root_i = 0; root_i < classes_->roots_.size(); root_i++)
      if(individual_vector.properties_on_[is_a_i] == classes_->roots_[root_i]->value_)
      {
        me->properties_on_.push_back(classes_->roots_[root_i]);
        i_find_my_properties_on = true;
      }

    //is a branch my class ?
    for(unsigned int branch_i = 0; branch_i < classes_->branchs_.size(); branch_i++)
      if(individual_vector.properties_on_[is_a_i] == classes_->branchs_[branch_i]->value_)
      {
        me->properties_on_.push_back(classes_->branchs_[branch_i]);
        i_find_my_properties_on = true;
      }

    //is a tmp mother is my class ?
    for(unsigned int branch_i  =0; branch_i < classes_->tmp_mothers_.size(); branch_i++)
      if(individual_vector.properties_on_[is_a_i] == classes_->tmp_mothers_[branch_i]->value_)
      {
        me->properties_on_.push_back(classes_->tmp_mothers_[branch_i]);
        i_find_my_properties_on = true;
      }

    //I create my class
    if(!i_find_my_properties_on)
    {
      ObjectVectors_t empty_vectors;
      classes_->add(individual_vector.properties_on_[is_a_i], empty_vectors);
      for(unsigned int root_i = 0; root_i < classes_->roots_.size(); root_i++)
        if(individual_vector.properties_on_[is_a_i] == classes_->roots_[root_i]->value_)
        {
          me->properties_on_.push_back(classes_->roots_[root_i]);
          i_find_my_properties_on = true;
        }
    }
  }

  /**********************
  ** Same In Individual
  **********************/
  //for all my inverses
  for(unsigned int same_i = 0; same_i < individual_vector.same_as_.size(); same_i++)
  {
    bool i_find_my_same = false;

    //is a root my inverse ?
    for(unsigned int individual_i = 0; individual_i < individuals_.size(); individual_i++)
      if(individual_vector.same_as_[same_i] == individuals_[individual_i]->value_)
      {
        me->same_as_.push_back(individuals_[individual_i]);
        individuals_[individual_i]->same_as_.push_back(me);
        i_find_my_same = true;
      }

    //I create my same
    if(!i_find_my_same)
    {
      IndividualBranch_t* my_same = new struct IndividualBranch_t(individual_vector.same_as_[same_i]);
      me->same_as_.push_back(my_same);
      my_same->same_as_.push_back(me);
      individuals_.push_back(my_same);
    }
  }
}

void IndividualGraph::add(std::vector<std::string>& distinct)
{
  for(unsigned int distinct_i = 0; distinct_i < distinct.size(); distinct_i++)
  {
    //I need to find myself
    IndividualBranch_t* me = nullptr;
    //Am I created ?
    for(unsigned int individual_i = 0; individual_i < individuals_.size(); individual_i++)
      if(distinct[distinct_i] == individuals_[individual_i]->value_)
        me = individuals_[individual_i];

    // I don't exist ?
    if(me == nullptr)
    {
      me = new struct IndividualBranch_t(distinct[distinct_i]);
      individuals_.push_back(me);
    }

    //for all my distincts ...
    for(unsigned int distinct_j = 0; distinct_j < distinct.size(); distinct_j++)
    {
      //... excepted me
      if(distinct_i != distinct_j)
      {
        bool i_find_my_distinct = false;

        //is my distinct created ?
        for(unsigned int individual_i = 0; individual_i < individuals_.size(); individual_i++)
          if(distinct[distinct_j] == individuals_[individual_i]->value_)
          {
            me->distinct_.push_back(individuals_[individual_i]);
            i_find_my_distinct = true;
          }

        //I create my distinct
        if(!i_find_my_distinct)
        {
          IndividualBranch_t* my_distinct = new struct IndividualBranch_t(distinct[distinct_j]);
          me->distinct_.push_back(my_distinct);
          individuals_.push_back(my_distinct);
        }
      }
    }
  }
}
