// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc2/introspection/state_traits.hpp>
#include <smacc2/smacc_event_generator.hpp>
#include <smacc2/smacc_state.hpp>
#include <smacc2/smacc_state_machine.hpp>
#include <smacc2/smacc_state_reactor.hpp>
#include <smacc2/smacc_tracing/trace_provider.hpp>

#define STATE_NAME (demangleSymbol(typeid(MostDerived).name()).c_str())

namespace smacc2
{
using namespace smacc2::introspection;
using namespace smacc2::default_events;

template <
  class MostDerived, class Context, class InnerInitial = mpl::list<>,
  sc::history_mode historyMode = sc::has_deep_history>
class SmaccState : public sc::simple_state<MostDerived, Context, InnerInitial, historyMode>,
                   public ISmaccState
{
  typedef sc::simple_state<MostDerived, Context, InnerInitial, historyMode> base_type;

public:
  typedef Context TContext;
  typedef typename Context::inner_context_type context_type;
  typedef typename context_type::state_iterator state_iterator;

  typedef InnerInitial LastDeepState;

  bool finishStateThrown;
  InnerInitial * smacc_inner_type;

  //////////////////////////////////////////////////////////////////////////
  struct my_context
  {
    my_context(typename base_type::context_ptr_type pContext) : pContext_(pContext) {}

    typename base_type::context_ptr_type pContext_;
  };

  SmaccState() = delete;

  // Constructor that initializes the state ros node handle
  SmaccState(my_context ctx)
  {
    static_assert(
      std::is_base_of<ISmaccState, Context>::value ||
        std::is_base_of<ISmaccStateMachine, Context>::value,
      "The context class must be a SmaccState or a SmaccStateMachine");
    static_assert(
      !std::is_same<MostDerived, Context>::value,
      "The context must be a different state or state machine "
      "than the current state");

    logger_.reset(new rclcpp::Logger(
      rclcpp::get_logger(smacc2::utils::cleanShortTypeName(typeid(MostDerived)))));

    RCLCPP_INFO(getLogger(), "[%s] creating state ", STATE_NAME);
    this->set_context(ctx.pContext_);

    node_ = this->getStateMachine().getNode();

    this->stateInfo_ = getStateInfo();

    // storing a reference to the parent state
    auto & ps = this->template context<Context>();
    parentState_ = dynamic_cast<ISmaccState *>(&ps);
    finishStateThrown = false;
  }

  virtual ~SmaccState() {}

  const smacc2::introspection::SmaccStateInfo * getStateInfo()
  {
    auto smInfo = this->getStateMachine().getStateMachineInfo();

    auto key = typeid(MostDerived).name();
    if (smInfo.states.count(key))
    {
      return smInfo.states[key].get();
    }
    else
    {
      return nullptr;
    }
  }

  std::string getName() override { return getShortName(); }

  std::string getFullName() { return demangleSymbol(typeid(MostDerived).name()); }

  std::string getShortName() { return smacc2::utils::cleanShortTypeName(typeid(MostDerived)); }

  virtual ISmaccState * getParentState()
  {
    // auto* ctx = dynamic_cast<ISmaccState*>(this->template context<Context *>());

    return parentState_;
  }

  // this function is called by boot statechart before the destructor call
  void exit()
  {
    auto * derivedThis = static_cast<MostDerived *>(this);
    this->getStateMachine().notifyOnStateExitting(derivedThis);
    {
      std::lock_guard<std::recursive_mutex> lock(this->getStateMachine().getMutex());
      this->getStateMachine().disposeStateAndDisconnectSignals();
      try
      {
        TRACEPOINT(smacc2_state_onExit_start, STATE_NAME);
        // static_cast<MostDerived *>(this)->onExit();
        standardOnExit(*derivedThis);
        TRACEPOINT(smacc2_state_onExit_end, STATE_NAME);
      }
      catch (...)
      {
      }
      this->getStateMachine().notifyOnStateExited(derivedThis);
    }
  }

public:
  // This method is static-polymorphic because of the curiously recurring template pattern. It
  // calls to the most derived class onEntry method if declared on smacc state construction
  void runtimeConfigure() {}

  // This method is static-polymorphic because of the curiously recurring template pattern. It
  // calls to the most derived class onEntry method if declared on smacc state construction
  void onEntry() {}

  // this method is static-polimorphic because of the curiously recurreing pattern. It
  // calls to the most derived class onExit method if declared on smacc state destruction
  void onExit() {}

  template <typename T>
  bool getGlobalSMData(std::string name, T & ret)
  {
    return base_type::outermost_context().getGlobalSMData(name, ret);
  }

  // Store globally in this state machine. (By value parameter )
  template <typename T>
  void setGlobalSMData(std::string name, T value)
  {
    base_type::outermost_context().setGlobalSMData(name, value);
  }

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *& storage)
  {
    base_type::outermost_context().requiresComponent(storage);
  }

  virtual ISmaccStateMachine & getStateMachine() { return base_type::outermost_context(); }

  template <typename TOrthogonal, typename TBehavior>
  static void configure_orthogonal_runtime(
    std::function<void(TBehavior & bh, MostDerived &)> initializationFunction)
  {
    configure_orthogonal_internal<TOrthogonal, TBehavior>([=](ISmaccState * state) {
      // auto bh = std::make_shared<TBehavior>(args...);
      auto bh = state->configure<TOrthogonal, TBehavior>();
      initializationFunction(*bh, *(static_cast<MostDerived *>(state)));
    });
  }

  template <typename TOrthogonal, typename TBehavior>
  static void configure_orthogonal_runtime(
    std::function<void(TBehavior & bh)> initializationFunction)
  {
    configure_orthogonal_internal<TOrthogonal, TBehavior>([=](ISmaccState * state) {
      // auto bh = std::make_shared<TBehavior>(args...);
      auto bh = state->configure<TOrthogonal, TBehavior>();
      initializationFunction(*bh);
    });
  }

  template <typename TOrthogonal, typename TBehavior, typename... Args>
  static void configure_orthogonal(Args &&... args)
  {
    configure_orthogonal_internal<TOrthogonal, TBehavior>([=](ISmaccState * state) {
      // auto bh = std::make_shared<TBehavior>(args...);
      state->configure<TOrthogonal, TBehavior>(args...);
    });
  }

  template <
    typename TStateReactor, typename TOutputEvent, typename TInputEventList, typename... TArgs>
  static std::shared_ptr<smacc2::introspection::StateReactorHandler> static_createStateReactor(
    TArgs... args)
  {
    auto srh = std::make_shared<smacc2::introspection::StateReactorHandler>(globalNh_);
    auto srinfo = std::make_shared<SmaccStateReactorInfo>();

    srinfo->stateReactorType = TypeInfo::getTypeInfoFromType<TStateReactor>();
    srinfo->outputEventType = TypeInfo::getTypeInfoFromType<TOutputEvent>();

    if (srinfo->outputEventType->templateParameters.size() == 2)
    {
      srinfo->objectTagType = srinfo->outputEventType->templateParameters[1];
    }
    else if (srinfo->outputEventType->templateParameters.size() == 1)
    {
      srinfo->objectTagType = TypeInfo::getTypeInfoFromType<state_reactors::EmptyObjectTag>();
    }
    else
    {
      assert(
        false &&
        "state reactor output events should have one or two parameters (SourceType, ObjectTag)");
    }

    // iterate statically on all event sources
    using boost::mpl::_1;
    using wrappedList = typename boost::mpl::transform<TInputEventList, _1>::type;
    AddTEventTypeStateReactorInfo<TInputEventList> op(srinfo.get());
    boost::mpl::for_each<wrappedList>(op);

    srinfo->srh = srh;
    srh->srInfo_ = srinfo;

    const std::type_info * tindex = &(typeid(MostDerived));  // get identifier of the current state

    if (!SmaccStateInfo::stateReactorsInfo.count(tindex))
      SmaccStateInfo::stateReactorsInfo[tindex] =
        std::vector<std::shared_ptr<SmaccStateReactorInfo>>();

    srinfo->factoryFunction = [&, srh, args...](ISmaccState * state) {
      auto sr =
        state->createStateReactor<TStateReactor, TOutputEvent, TInputEventList, TArgs...>(args...);
      srh->configureStateReactor(sr);
      sr->initialize(state);
      return sr;
    };

    SmaccStateInfo::stateReactorsInfo[tindex].push_back(srinfo);

    return srh;
  }

  template <typename TEventGenerator, typename... TUArgs>
  static std::shared_ptr<smacc2::introspection::EventGeneratorHandler> static_createEventGenerator(
    TUArgs... args)
  {
    auto egh = std::make_shared<smacc2::introspection::EventGeneratorHandler>();
    auto eginfo = std::make_shared<SmaccEventGeneratorInfo>();
    eginfo->eventGeneratorType = TypeInfo::getTypeInfoFromType<TEventGenerator>();

    eginfo->egh = egh;
    egh->egInfo_ = eginfo;

    const std::type_info * tindex = &(typeid(MostDerived));  // get identifier of the current state

    if (!SmaccStateInfo::eventGeneratorsInfo.count(tindex))
      SmaccStateInfo::eventGeneratorsInfo[tindex] =
        std::vector<std::shared_ptr<SmaccEventGeneratorInfo>>();

    eginfo->factoryFunction = [&, egh, args...](ISmaccState * state) {
      auto eg = state->createEventGenerator<TEventGenerator>(args...);
      egh->configureEventGenerator(eg);
      eg->initialize(state);
      eg->template onStateAllocation<MostDerived, TEventGenerator>();
      return eg;
    };

    SmaccStateInfo::eventGeneratorsInfo[tindex].push_back(eginfo);
    return egh;
  }

  template <typename TStateReactor, typename... TUArgs>
  static std::shared_ptr<smacc2::introspection::StateReactorHandler> static_createStateReactor_aux(
    TUArgs... args)
  {
    auto srh = std::make_shared<smacc2::introspection::StateReactorHandler>(globalNh_);
    auto srinfo = std::make_shared<SmaccStateReactorInfo>();

    srinfo->stateReactorType = TypeInfo::getTypeInfoFromType<TStateReactor>();
    srinfo->srh = srh;
    srh->srInfo_ = srinfo;

    const std::type_info * tindex = &(typeid(MostDerived));  // get identifier of the current state

    if (!SmaccStateInfo::stateReactorsInfo.count(tindex))
      SmaccStateInfo::stateReactorsInfo[tindex] =
        std::vector<std::shared_ptr<SmaccStateReactorInfo>>();

    srinfo->factoryFunction = [&, srh, args...](ISmaccState * state) {
      auto sr = state->createStateReactor<TStateReactor>(args...);
      srh->configureStateReactor(sr);
      sr->initialize(state);
      return sr;
    };

    SmaccStateInfo::stateReactorsInfo[tindex].push_back(srinfo);

    return srh;
  }

  void checkWhileLoopConditionAndThrowEvent(bool (MostDerived::*conditionFn)())
  {
    auto * thisobject = static_cast<MostDerived *>(this);
    auto condition = boost::bind(conditionFn, thisobject);
    bool conditionResult = condition();

    // RCLCPP_INFO("LOOP EVENT CONDITION: %d", conditionResult);
    if (conditionResult)
    {
      this->postEvent<EvLoopContinue<MostDerived>>();
    }
    else
    {
      this->postEvent<EvLoopEnd<MostDerived>>();
    }
    RCLCPP_INFO(getLogger(), "[%s] POST THROW CONDITION", STATE_NAME);
  }

  void throwSequenceFinishedEvent() { this->postEvent<EvSequenceFinished<MostDerived>>(); }

  //////////////////////////////////////////////////////////////////////////
  // The following declarations should be private.
  // They are only public because many compilers lack template friends.
  //////////////////////////////////////////////////////////////////////////
  // See base class for documentation
  typedef typename base_type::outermost_context_base_type outermost_context_base_type;
  typedef typename base_type::inner_context_ptr_type inner_context_ptr_type;
  typedef typename base_type::context_ptr_type context_ptr_type;
  typedef typename base_type::inner_initial_list inner_initial_list;

  static void initial_deep_construct(outermost_context_base_type & outermostContextBase)
  {
    deep_construct(&outermostContextBase, outermostContextBase);
  }

  // See base class for documentation
  static void deep_construct(
    const context_ptr_type & pContext, outermost_context_base_type & outermostContextBase)
  {
    const inner_context_ptr_type pInnerContext(shallow_construct(pContext, outermostContextBase));
    base_type::template deep_construct_inner<inner_initial_list>(
      pInnerContext, outermostContextBase);
  }

  static inner_context_ptr_type shallow_construct(
    const context_ptr_type & pContext, outermost_context_base_type & outermostContextBase)
  {
    // allocating in memory
    auto state = new MostDerived(
      SmaccState<MostDerived, Context, InnerInitial, historyMode>::my_context(pContext));
    const inner_context_ptr_type pInnerContext(state);

    TRACEPOINT(smacc2_state_onEntry_start, STATE_NAME);
    state->entryStateInternal();
    TRACEPOINT(smacc2_state_onEntry_end, STATE_NAME);

    outermostContextBase.add(pInnerContext);
    return pInnerContext;
  }

private:
  template <typename TOrthogonal, typename TBehavior>
  static void configure_orthogonal_internal(
    std::function<void(ISmaccState * state)> initializationFunction)
  {
    ClientBehaviorInfoEntry bhinfo;
    bhinfo.factoryFunction = initializationFunction;

    bhinfo.behaviorType = &(typeid(TBehavior));
    bhinfo.orthogonalType = &(typeid(TOrthogonal));

    const std::type_info * tindex = &(typeid(MostDerived));
    if (!SmaccStateInfo::staticBehaviorInfo.count(tindex))
      SmaccStateInfo::staticBehaviorInfo[tindex] = std::vector<ClientBehaviorInfoEntry>();

    SmaccStateInfo::staticBehaviorInfo[tindex].push_back(bhinfo);
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("static"), "[states walking] State "
                                      << smacc2::utils::cleanShortTypeName(*tindex)
                                      << "client behavior count: "
                                      << SmaccStateInfo::staticBehaviorInfo[tindex].size());
  }

  void entryStateInternal()
  {
    // finally we go to the derived state onEntry Function

    RCLCPP_INFO(getLogger(), "[%s] State object created. Initializating...", STATE_NAME);
    this->getStateMachine().notifyOnStateEntryStart(static_cast<MostDerived *>(this));

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << smacc2::utils::cleanShortTypeName(typeid(MostDerived)).c_str()
                       << "] creating ros subnode");

    // before dynamic runtimeConfigure, we execute the staticConfigure behavior configurations
    {
      RCLCPP_INFO(getLogger(), "[%s] -- STATIC STATE DESCRIPTION --", STATE_NAME);

      for (const auto & stateReactorsVector : SmaccStateInfo::staticBehaviorInfo)
      {
        RCLCPP_DEBUG(
          getLogger(), "[%s] state reactor info: %s", STATE_NAME,
          demangleSymbol(stateReactorsVector.first->name()).c_str());
        for (auto & srinfo : stateReactorsVector.second)
        {
          RCLCPP_DEBUG(
            getLogger(), "[%s] state reactor: %s", STATE_NAME,
            demangleSymbol(srinfo.behaviorType->name()).c_str());
        }
      }

      const std::type_info * tindex = &(typeid(MostDerived));
      auto & staticDefinedBehaviors = SmaccStateInfo::staticBehaviorInfo[tindex];
      auto & staticDefinedStateReactors = SmaccStateInfo::stateReactorsInfo[tindex];
      auto & staticDefinedEventGenerators = SmaccStateInfo::eventGeneratorsInfo[tindex];

      RCLCPP_DEBUG_STREAM(
        getLogger(), "finding static client behaviors. State Database: "
                       << SmaccStateInfo::staticBehaviorInfo.size() << ". Current state "
                       << cleanShortTypeName(*tindex)
                       << " cbs: " << SmaccStateInfo::staticBehaviorInfo[tindex].size());
      for (auto & bhinfo : staticDefinedBehaviors)
      {
        RCLCPP_INFO(
          getLogger(), "[%s] Creating static client behavior: %s", STATE_NAME,
          demangleSymbol(bhinfo.behaviorType->name()).c_str());
        bhinfo.factoryFunction(this);
      }

      for (auto & sr : staticDefinedStateReactors)
      {
        RCLCPP_INFO(
          getLogger(), "[%s] Creating static state reactor: %s", STATE_NAME,
          sr->stateReactorType->getFullName().c_str());
        sr->factoryFunction(this);
      }

      for (auto & eg : staticDefinedEventGenerators)
      {
        RCLCPP_INFO(
          getLogger(), "[%s] Creating static event generator: %s", STATE_NAME,
          eg->eventGeneratorType->getFullName().c_str());
        eg->factoryFunction(this);
      }

      RCLCPP_INFO(getLogger(), "[%s] ---- END STATIC DESCRIPTION", STATE_NAME);
    }

    RCLCPP_INFO(getLogger(), "[%s] State runtime configuration", STATE_NAME);

    auto * derivedthis = static_cast<MostDerived *>(this);

    // second the orthogonals are internally configured
    this->getStateMachine().notifyOnRuntimeConfigured(derivedthis);

    TRACEPOINT(smacc2_state_onRuntimeConfigure_start, STATE_NAME);
    // first we runtime configure the state, where we create client behaviors
    static_cast<MostDerived *>(this)->runtimeConfigure();
    TRACEPOINT(smacc2_state_onRuntimeConfigure_end, STATE_NAME);

    this->getStateMachine().notifyOnRuntimeConfigurationFinished(derivedthis);

    RCLCPP_INFO(getLogger(), "[%s] State OnEntry", STATE_NAME);

    static_cast<MostDerived *>(this)->onEntry();

    // here orthogonals and client behaviors are entered OnEntry
    this->getStateMachine().notifyOnStateEntryEnd(derivedthis);
  }
};
}  // namespace smacc2
