#pragma once

#include <cmath>
#include <chrono>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;
class CpVehicleLocalPosition;

class CbOrbitLocation : public smacc2::SmaccAsyncClientBehavior, public smacc2::ISmaccUpdatable
{
public:
  CbOrbitLocation(
    float centerX, float centerY, float altitude,
    float radius = 5.0f, float angularVelocity = 0.5f, int numOrbits = 3);

  void onEntry() override;
  void onExit() override;
  void update() override;

private:
  float centerX_;
  float centerY_;
  float altitude_;
  float radius_;
  float angularVelocity_;
  int numOrbits_;

  float currentAngle_ = 0.0f;
  float startAngle_ = 0.0f;
  std::chrono::steady_clock::time_point lastUpdateTime_;

  CpTrajectorySetpoint * trajectorySetpoint_ = nullptr;
  CpVehicleLocalPosition * localPosition_ = nullptr;
};

}  // namespace cl_px4_mr
