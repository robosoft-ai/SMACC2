/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <algorithm>
#include <smacc/introspection/introspection.h>
#include <boost/statechart/event.hpp>
#include <map>

namespace smacc
{
    class ISmaccState;
    class ISMaccStateMachine;

    class SmaccEventGenerator
    {
    public:
        SmaccEventGenerator();
        virtual ~SmaccEventGenerator();

        template <typename TState, typename TSource>
        void onStateAllocation();

        virtual void onEntry();
        virtual void onExit();

        template <typename EventType>
        void postEvent(const EventType &ev);

        template <typename EventType>
        void postEvent();

        void initialize(ISmaccState *ownerState);
        virtual void onInitialized();

    private:
        ISmaccState *ownerState_;
        friend ISmaccStateMachine;
    };

} // namespace smacc