/**
 * \file arb_subject_ctrl_callback.cpp
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


// callback for subject ctrl state msg
void ArbSubjectCtrl::callbackSubjectCtrlStateMsg(const boost::shared_ptr<topic_arbitration::topic_source const> &subject_ctrl_state_msg)
{
  subject_ctrl_state_msg_ = *subject_ctrl_state_msg;
}


// callback for shape shifter msg
void ArbSubjectCtrl::callbackShapeShifterMsg(const boost::shared_ptr<topic_tools::ShapeShifter const> &shape_shifter_msg, const vector<uint8_t> &state_mappings)
{
  for(uint32_t ii = 0; ii < state_mappings.size(); ii++)
    {
      if(subject_ctrl_state_msg_.state == state_mappings[ii])
        {
          if(!is_shape_shifter_advertised_)
            {
              shape_shifter_pub_ = shape_shifter_msg->advertise(nh_, "shape_shifter", 10);
              is_shape_shifter_advertised_ = true;
            }
          shape_shifter_pub_.publish(*shape_shifter_msg);
          return;
        }
    }
}
