/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/smacc_client_behavior_base.h>
#include <smacc/smacc_signal.h>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <future>

namespace smacc
{
    template <typename AsyncCB, typename Orthogonal>
    struct EvCbFinished : sc::event<EvCbFinished<AsyncCB, Orthogonal>>
    {
    };

    template <typename AsyncCB, typename Orthogonal>
    struct EvCbSuccess : sc::event<EvCbSuccess<AsyncCB, Orthogonal>>
    {
    };

    template <typename AsyncCB, typename Orthogonal>
    struct EvCbFailure : sc::event<EvCbFailure<AsyncCB, Orthogonal>>
    {
    };

    // Asnchronous client behaviors are used when the onEntry or onExit function execution is slow
    // CONCEPT: this funcionality is related with the orthogonality of SmaccState machines.
    // No behavior should block the creation of other behaviors, all of them conceptually start in parallel.
    // Alternative for long duration behaviors: using default-synchromous SmaccClientBehaviors with the update method
    // ASYNCHRONOUS STATE MACHINES DESIGN NOTES: Asynchromous behaviors can safely post events and use its local methods,
    //  but the interaction with other components or elements of
    // the state machine is not by-default thread safe and must be manually implemented. For example, if some element of the architecture
    // (components, states, clients) need to access to this behavior client information it is needed to implement a mutex for the internal
    // state of this behavior. Other example: if this behavior access to some component located in other thread, it is also may be needed
    // to some mutex for that component
    class SmaccAsyncClientBehavior : public ISmaccClientBehavior
    {
    public:
        template <typename TOrthogonal, typename TSourceObject>
        void onOrthogonalAllocation();

        virtual ~SmaccAsyncClientBehavior();

        template <typename TCallback, typename T>
        boost::signals2::connection onSuccess(TCallback callback, T *object);

        template <typename TCallback, typename T>
        boost::signals2::connection onFinished(TCallback callback, T *object);

        template <typename TCallback, typename T>
        boost::signals2::connection onFailure(TCallback callback, T *object);

    protected:
        virtual void executeOnEntry() override;
        virtual void executeOnExit() override;

        void postSuccessEvent();
        void postFailureEvent();

        virtual void dispose() override;

    private:
        std::future<int> onEntryThread_;
        std::future<int> onExitThread_;

        std::function<void()> postFinishEventFn_;
        std::function<void()> postSuccessEventFn_;
        std::function<void()> postFailureEventFn_;

        SmaccSignal<void()> onFinished_;
        SmaccSignal<void()> onSuccess_;
        SmaccSignal<void()> onFailure_;
    };
} // namespace smacc

#include <smacc/impl/smacc_asynchronous_client_behavior_impl.h>