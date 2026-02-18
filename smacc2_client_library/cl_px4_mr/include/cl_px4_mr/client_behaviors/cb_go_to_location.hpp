#pragma once

#include <cmath>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;
class CpGoalChecker;

class CbGoToLocation : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbGoToLocation(
    float targetX, float targetY, float targetZ,
    float yaw = std::numeric_limits<float>::quiet_NaN());

  void onEntry() override;
  void onExit() override;

private:
  void onGoalReachedCallback();

  float targetX_;
  float targetY_;
  float targetZ_;
  float yaw_;
  CpTrajectorySetpoint * trajectorySetpoint_ = nullptr;
  CpGoalChecker * goalChecker_ = nullptr;
};

}  // namespace cl_px4_mr
