/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <smacc/smacc_event_generator.h>

namespace smacc
{

    SmaccEventGenerator::SmaccEventGenerator()
    {
    }

    SmaccEventGenerator::~SmaccEventGenerator()
    {
    }

    void SmaccEventGenerator::onEntry()
    {
    }

    void SmaccEventGenerator::onExit()
    {
    }

    void SmaccEventGenerator::initialize(smacc::ISmaccState *ownerState)
    {
        this->ownerState_ = ownerState;
        this->onInitialized();
    }

    void SmaccEventGenerator::onInitialized()
    {
    }

} // namespace smacc