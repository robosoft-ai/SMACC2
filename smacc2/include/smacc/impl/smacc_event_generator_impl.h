/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/smacc_event_generator.h>
#include <smacc/introspection/introspection.h>

namespace smacc
{
    template <typename EventType>
    void SmaccEventGenerator::postEvent(const EventType &ev)
    {
        ownerState_->postEvent(ev);
    }

    template <typename EventType>
    void SmaccEventGenerator::postEvent()
    {
        ownerState_->postEvent<EventType>();
    }

    template <typename TState, typename TSource>
    void SmaccEventGenerator::onStateAllocation()
    {
    }
} // namespace smacc