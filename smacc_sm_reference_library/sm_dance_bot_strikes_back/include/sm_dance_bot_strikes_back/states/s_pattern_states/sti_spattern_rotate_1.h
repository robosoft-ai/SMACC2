namespace sm_dance_bot_strikes_back
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternRotate1 : smacc::SmaccState<StiSPatternRotate1, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiSPatternForward1>,
        Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StiSPatternLoopStart>

        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrNavigation, CbAbsoluteRotate>();
        configure_orthogonal<OrLED, CbLEDOff>();
    }

    void runtimeConfigure()
    {
        auto &superstate = this->context<SS>();
        RCLCPP_INFO(getNode()->get_logger(), "[StiSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d", superstate.iteration_count, SS::total_iterations());

        double offset;
        // if (superstate.iteration_count == 1)
        // {
            offset = 0;
        // }
        // else
        // {
        //     offset = 13.5;
        // }

        auto absoluteRotateBehavior = this->getOrthogonal<OrNavigation>()
                                          ->getClientBehavior<CbAbsoluteRotate>();

        if (superstate.direction() == TDirection::RIGHT)
        {
            // - offset because we are looking to the north and we have to turn clockwise
            absoluteRotateBehavior->absoluteGoalAngleDegree = superstate.initialStateAngle - offset;
        }
        else
        {
            // - offset because we are looking to the south and we have to turn counter-clockwise
            absoluteRotateBehavior->absoluteGoalAngleDegree = superstate.initialStateAngle + offset;
        }
    }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot_strikes_back