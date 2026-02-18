#pragma once

#include <cmath>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleLocalPosition;

class CpGoalChecker : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpGoalChecker();
  virtual ~CpGoalChecker();

  void onInitialize() override;
  void update() override;

  void setGoal(float x, float y, float z, float xy_tolerance = 0.5f, float z_tolerance = 0.3f);
  void clearGoal();
  bool isGoalActive() const;

  smacc2::SmaccSignal<void()> onGoalReached_;

private:
  CpVehicleLocalPosition * localPosition_ = nullptr;
  float goalX_ = 0.0f;
  float goalY_ = 0.0f;
  float goalZ_ = 0.0f;
  float xyTolerance_ = 0.5f;
  float zTolerance_ = 0.3f;
  bool goalActive_ = false;
};

}  // namespace cl_px4_mr
