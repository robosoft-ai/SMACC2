#include <smacc/smacc.h>
namespace sm_dance_bot
{
// STATE DECLARATION
struct StEventCountDown : smacc::SmaccState<StEventCountDown, MsDanceBotRunMode>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvCountdownEnd<SrEventCountdown>, StNavigateToWaypointsX>,
    Transition<EvGlobalError, MsDanceBotRecoveryMode>
    
    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        //   configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
        //   configure_orthogonal<OrStringPublisher, CbStringPublisher>("Hello World!");
        //   configure_orthogonal<OrTemperatureSensor, CbConditionTemperatureSensor>();
        //   configure_orthogonal<OrService3, CbService3>(Service3Command::SERVICE3_ON);      
        
        // Create State Reactor
        
        //auto srCountdown = static_createStateReactor<SrEventCountdown>(5);        
        //srCountdown->addInputEvent<EvTimer<ClRosTimer, OrTimer>>();
        //srCountdown->setOutputEvent<EvCountdownEnd<SrEventCountdown>>();

        auto srCountdown = static_createStateReactor<SrEventCountdown, 
                                                         EvCountdownEnd<SrEventCountdown>, 
                                                         mpl::list<
                                                                  EvTimer<ClRosTimer, OrTimer>
                                                                  >>(5);
    }
};
}