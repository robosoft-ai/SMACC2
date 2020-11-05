#pragma once
#include <smacc/common.h>
#include <smacc/smacc_state_reactor.h>
#include <map>
#include <typeinfo>
#include <boost/statechart/event.hpp>

namespace smacc
{
namespace state_reactors
{
template <typename TSource, typename TObjectTag = EmptyObjectTag>
struct Evsr_conditionalTrue : sc::event<Evsr_conditionalTrue<TSource, TObjectTag>>
{
};

//-----------------------------------------------------------------------
class Srsr_conditional : public StateReactor
{
private:
    std::map<const std::type_info *, bool> triggeredEvents;
    bool conditionFlag;

public:
    template <typename TEv>
    Srsr_conditional(std::function<bool(TEv *)> sr_conditionalFunction)
    {
        std::function<void(TEv *)> callback =
            [=](TEv *ev) {
                bool condition = sr_conditionalFunction(ev);
                this->conditionFlag = condition;
            };

        this->createEventCallback(callback);
    }

    ~Srsr_conditional();

    virtual bool triggers() override;
};
} // namespace state_reactors
} // namespace smacc