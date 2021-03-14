namespace sm_dance_bot_strikes_back
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternStartLoop : smacc::SmaccState<StiFPatternStartLoop<SS>, SS>
{
  typedef SmaccState<StiFPatternStartLoop<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
      Transition<EvLoopContinue<StiFPatternStartLoop<SS>>, StiFPatternForward2<SS>, CONTINUELOOP>>
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
  }

  bool loopCondition()
  {
    cl_lidar::ClLidarSensor *lidarClient;
    this->requiresClient(lidarClient);

    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    auto horizontalDistance = lidarData->forwardObstacleDistance;

    return horizontalDistance > 0.5 /*meters*/; // go ahead until 1.5m before the wall
  }

  void onEntry()
  {
    TSti::checkWhileLoopConditionAndThrowEvent(&StiFPatternStartLoop<SS>::loopCondition);
  }
};
} // namespace f_pattern_states
} // namespace sm_dance_bot_strikes_back