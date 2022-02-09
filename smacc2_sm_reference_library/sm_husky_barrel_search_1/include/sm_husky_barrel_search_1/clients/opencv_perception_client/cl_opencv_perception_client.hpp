#pragma once

#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <std_msgs/msg/int32.hpp>

namespace sm_husky_barrel_search_1
{
namespace cl_opencv_perception
{
class ClOpenCVPerception : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::Int32>
{
public:
  
  ClOpenCVPerception(std::string topicname = "/detected_color")
    : smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::Int32>(topicname)
  {
  }

  virtual ~ClOpenCVPerception()
  {
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::Int32>::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }
};
}  
}  