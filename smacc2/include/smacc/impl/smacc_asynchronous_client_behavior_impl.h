/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/smacc_asynchronous_client_behavior.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{

    template <typename TOrthogonal, typename TSourceObject>
    void SmaccAsyncClientBehavior::onOrthogonalAllocation()
    {
        postFinishEventFn_ = [=] {
            this->onFinished_();
            this->postEvent<EvCbFinished<TSourceObject, TOrthogonal>>();
        };

        postSuccessEventFn_ = [=] {
            this->onSuccess_();
            this->postEvent<EvCbSuccess<TSourceObject, TOrthogonal>>();
        };

        postFailureEventFn_ = [=] {
            this->onFailure_();
            this->postEvent<EvCbFailure<TSourceObject, TOrthogonal>>();
        };
    }

    template <typename TCallback, typename T>
    boost::signals2::connection SmaccAsyncClientBehavior::onSuccess(TCallback callback, T *object)
    {
        return this->getStateMachine()->createSignalConnection(onSuccess_, callback, object);
    }

    template <typename TCallback, typename T>
    boost::signals2::connection SmaccAsyncClientBehavior::onFinished(TCallback callback, T *object)
    {
        return this->getStateMachine()->createSignalConnection(onFinished_, callback, object);
    }

    template <typename TCallback, typename T>
    boost::signals2::connection SmaccAsyncClientBehavior::onFailure(TCallback callback, T *object)
    {
        return this->getStateMachine()->createSignalConnection(onFailure_, callback, object);
    }
}