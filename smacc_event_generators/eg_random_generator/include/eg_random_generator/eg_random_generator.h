#pragma once
#include <smacc/common.h>
#include <smacc/smacc_event_generator.h>
#include <smacc/smacc_updatable.h>
#include <typeinfo>
#include <boost/statechart/event.hpp>
#include <functional>

namespace smacc
{
    namespace state_reactors
    {
        template <typename TSource, typename TState>
        struct EventA : sc::event<EventA<TSource, TState>>
        {
        };

        template <typename TSource, typename TState>
        struct EventB : sc::event<EventB<TSource, TState>>
        {
        };

        template <typename TSource, typename TState>
        struct EventC : sc::event<EventC<TSource, TState>>
        {
        };

        enum class RandomGenerateReactorMode
        {
            INPUT_EVENT_TRIGGERED,
            ONE_SHOT,
            ON_UPDATE
        };

        //-----------------------------------------------------------------------
        class EgRandomGenerator : public SmaccEventGenerator, public ISmaccUpdatable
        {
        public:
            EgRandomGenerator(RandomGenerateReactorMode mode, double evAMin = 1, double evAMax = 4, double evBMin = 5, double evBMax = 8, double evCMin = 9, double evCMax = 12);

            virtual void onEntry() override;

            template <typename TState, typename TSource>
            void onStateAllocation()
            {
                this->postEventA = [this]() { this->postEvent<EventA<TSource, TState>>(); };
                this->postEventB = [this]() { this->postEvent<EventB<TSource, TState>>(); };
                this->postEventC = [this]() { this->postEvent<EventC<TSource, TState>>(); };
            }

            void postRandomEvents();

            virtual void update() override;

            RandomGenerateReactorMode mode_;

        private:
            std::function<void()> postEventA;
            std::function<void()> postEventB;
            std::function<void()> postEventC;

            double evAMin_;
            double evAMax_;
            double evBMin_;
            double evBMax_;
            double evCMin_;
            double evCMax_;

            double minValue;
            double maxValue;
        };
    } // namespace state_reactors
} // namespace smacc