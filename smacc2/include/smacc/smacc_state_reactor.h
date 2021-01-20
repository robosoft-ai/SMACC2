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

namespace state_reactors
{
struct EmptyObjectTag
{
};
} // namespace state_reactors

class StateReactor
{
public:
    ISmaccState *ownerState;
    std::function<void()> postEventFn;
    std::vector<const std::type_info *> eventTypes;
    std::map<const std::type_info *, std::function<void(void *)>> eventCallbacks_;

    StateReactor();
    virtual ~StateReactor();

    virtual void onInitialized();

    virtual void onEntry();
    virtual void onExit();

    virtual void onEventNotified(const std::type_info *eventType);

    template <typename EventType>
    void postEvent(const EventType &ev);

    template <typename EventType>
    void postEvent();

    // type based event callback
    template <typename T, typename TClass>
    void createEventCallback(void (TClass::*callback)(T *), TClass *object);

    // type based event callback
    template <typename T>
    void createEventCallback(std::function<void(T *)> callback);

    void update();

    //must returns true when the output event is triggered
    virtual bool triggers() = 0;

    template <typename TEv>
    void addInputEvent();

    template <typename TEv>
    void setOutputEvent();

    //TDerived
    void initialize(ISmaccState *ownerState);

    rclcpp::Node::SharedPtr getNode();

private:
    friend ISmaccStateMachine;

    template <typename TEvent>
    void notifyEvent(TEvent *ev)
    {
        //the state machine uses this method to notify this state reactor some event happened.
        auto tid = &(typeid(TEvent));
        if (std::find(eventTypes.begin(), eventTypes.end(), tid) != eventTypes.end())
        {
            this->onEventNotified(tid);
            this->update();

            if (eventCallbacks_.count(tid))
            {
                eventCallbacks_[tid]((void *)ev);
            }
        }
    }
};

} // namespace smacc
