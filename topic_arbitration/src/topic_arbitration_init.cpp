/**
 * \file arb_subject_ctrl_init.cpp
 *
 * \author Brian J. Julian
 *
 * \version 0.1
 *
 * \date 01 May 2011
 *
 */


// includes
#include <topic_arbitration.h>


// init params
void ArbSubjectCtrl::initParams(void)
{
  // get ros param
  private_nh_.param("state_mappings", state_mappings_, state_mappings_);

  // volatile bools
  is_shape_shifter_advertised_ = false;
  //received_subject_ctrl_state_msg_ = false;
  
  subject_ctrl_state_msg_.state = 0;
  received_subject_ctrl_state_msg_ = true;
}


// init publishers
void ArbSubjectCtrl::initPublishers(void)
{
  // needs to be published using advertise helper (see shape shifter callback)
  //  shape_shifter_pub_ = nh_.advertise<topic_tools::ShapeShifter>("shape_shifter", 10, true);
}


// init subscribers
void ArbSubjectCtrl::initSubscribers(void)
{
  subject_ctrl_state_sub_ = nh_.subscribe<topic_arbitration::topic_source>("topic_source", 10, &ArbSubjectCtrl::callbackSubjectCtrlStateMsg, this);

  if(state_mappings_.valid())
    {
      for(int32_t ii = 0; ii < state_mappings_.size(); ii++)
        {
          vector<uint8_t> state_value_mappings;
          for(int32_t jj = 0; jj < state_mappings_[ii][1].size(); jj++)
            {
              state_value_mappings.push_back(uint8_t(int32_t(state_mappings_[ii][1][jj])));
            }
          shape_shifter_subs_.push_back(nh_.subscribe<topic_tools::ShapeShifter>(state_mappings_[ii][0], 10, boost::bind(&ArbSubjectCtrl::callbackShapeShifterMsg, this,  _1, state_value_mappings)));
        }
    }
}
