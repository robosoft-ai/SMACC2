#pragma once

#include <smacc2/component.hpp>

namespace cl_move_base_z
{
class TrajectoryStorage : public smacc2::ISmaccComponent
{
public:
  TrajectoryStorage();

  void onInitialize() override;

private :

    std::mutex m_mutex_;
    
};

}  // namespace cl_move_base_z
