namespace sm_dance_bot
{
namespace s_pattern_states
{

 template<class TDerived, typename TContext>
  struct B: smacc::SmaccState<TDerived, TContext>
  {
    public:
        using SmaccState<TDerived, TContext>::SmaccState;
        //typedef typename TDerived::reactions reactions;
  };

// STATE DECLARATION
struct StiSPatternLoopStart : public B<StiSPatternLoopStart, SS>
{
  using B::B;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvLoopContinue<StiSPatternLoopStart>, StiSPatternRotate1, CONTINUELOOP>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }

  bool loopCondition()
  {
    auto &superstate = this->context<SS>();
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    checkWhileLoopConditionAndThrowEvent(&StiSPatternLoopStart::loopCondition);
  }
};


} // namespace s_pattern_states
} // namespace sm_dance_bot
