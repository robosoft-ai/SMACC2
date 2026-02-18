#include <cl_px4_mr/components/cp_goal_checker.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

namespace cl_px4_mr
{

CpGoalChecker::CpGoalChecker() {}

CpGoalChecker::~CpGoalChecker() {}

void CpGoalChecker::onInitialize()
{
  this->requiresComponent(localPosition_);
  RCLCPP_INFO(getLogger(), "CpGoalChecker: initialized");
}

void CpGoalChecker::update()
{
  if (!goalActive_ || !localPosition_ || !localPosition_->isValid()) return;

  float dx = localPosition_->getX() - goalX_;
  float dy = localPosition_->getY() - goalY_;
  float dz = localPosition_->getZ() - goalZ_;
  float xyDist = std::sqrt(dx * dx + dy * dy);
  float zDist = std::abs(dz);

  if (xyDist <= xyTolerance_ && zDist <= zTolerance_)
  {
    RCLCPP_INFO(getLogger(),
      "CpGoalChecker: GOAL REACHED (xy_dist=%.2f z_dist=%.2f)", xyDist, zDist);
    goalActive_ = false;
    onGoalReached_();
  }
}

void CpGoalChecker::setGoal(float x, float y, float z, float xy_tolerance, float z_tolerance)
{
  goalX_ = x;
  goalY_ = y;
  goalZ_ = z;
  xyTolerance_ = xy_tolerance;
  zTolerance_ = z_tolerance;
  goalActive_ = true;
  RCLCPP_INFO(getLogger(),
    "CpGoalChecker: goal set [%.2f, %.2f, %.2f] tol(xy=%.2f z=%.2f)",
    x, y, z, xy_tolerance, z_tolerance);
}

void CpGoalChecker::clearGoal()
{
  goalActive_ = false;
}

bool CpGoalChecker::isGoalActive() const
{
  return goalActive_;
}

}  // namespace cl_px4_mr
