## Introduction

This is a mostly chronological writing of my learing process of [SMACC2](https://github.com/robosoft-ai/SMACC2). It is only slightly redacted and hence by no means an ideal guideline to learning SMACC2, and it is **unfinished**, as time constraints forced me to stop and focus on other priorities. Yet I decided to release this file, as I think it can be valuable to anyone new to SMACC2 / Boost Statechart, and hopefully significantly accelerate their (initial) learning process. The text is structured in sections that describe a part of the code, each followed by 'Intermediate conclusions' that give a concise resume of that part of the code.

Feel free to correct me on topics that I might have misunderstood, or to elaborate on topics that might benefit from further clarification.
</br>
</br>


## Boost Statechart

In order to understand SMACC2, you need to have a thorough understanding of the Boost Statechart library.

At the bare minimum, read through [these tutorials](https://www.boost.org/doc/libs/1_81_0/libs/statechart/doc/tutorial.html).</br>
[This eBook](https://github.com/CodeSports/State-Machine-Using-Boost-Statechart) is also a good resource.
</br>
</br>
The source code for the Boost Statechart tutorial examples can be found by [downloading Boost](https://www.boost.org/users/download/) or on [github](https://github.com/boostorg/statechart/tree/master/example).


</br>

### Some concepts worth repeating for reference:

But seriously: **[read those tutorials first!](https://www.boost.org/doc/libs/1_81_0/libs/statechart/doc/tutorial.html)**
</br>
</br>
- Boost Statechart extensively uses **`structs`**. There is no functional difference between a `struct` and a `class`. Structs can also have constructors, destructors, public and private member variables or functions, etc. And structs can also inherit from other classes or structs. The only functional difference between a struct and a class is the following: if no access specifiers (public/private/protected) are specified, classes will default to `private`, whereas structs will default to `public`. For instance:

  ```c++
  struct MyStruct : BaseClass { ... };   // identical to "struct MyStruct : public BaseClass { ... };"
  class MyClass : BaseClass { ... };   // identical to "class MyClass : private BaseClass { ... };"
  ```
  
  </br>
- Boost Statechart extensively uses the **curiously recurring template pattern** (CRTP), which means that a derived class/struct inherits from a templated class/struct that has the derived class/struct as its first template parameter. E.g.:

  ```c++
  struct Mystruct : TemplateBaseClass< MyStruct >
  ```
  This is curious indeed! ;-)/br>
  Wrt. the use of SMACC2 / Boost Statechart, there is no need to further study this design pattern; just accept it for what it is and get used to the notation.

  </br>
- Typically, the **namespace** `sc` is defined for easier readability:

  ```c++
  namespace sc = boost::statechart;
  ```
  
  </br>
- A Boost Statechart **state machine** inherits from `sc::state_machine` (synchronous state machine), or from `sc::asynchronous_state_machine` e.g.:

  ```c++
  struct ActiveState;
  struct SyncStateMachine : sc::state_machine<SyncStateMachine, ActiveState>
  struct AsyncStateMachine : sc::asynchronous_state_machine<AsyncStateMachine, ActiveState>
  ```

  In this, `ActiveState` is forward declared, as it is needed in the declaration of the state machines.</br>
  Re. the state machine declarations: according to the CRTP, the first template parameter in the state machine declaration is the derived type. The second template parameter is the (type of) state which will be activated upon entering the state machine.</br>
  
   
  — A Smacc2 state machine inherits from `SmaccStateMachineBase`, which is derived from `sc::asynchronous_state_machine`.
   </br>
   </br>
   
- A Boost Statechart **state** inherits from `sc::simple_state` (or `sc::state`), e.g.:
  ```c++
  struct ActiveState : sc::simple_state<ActiveState, StateMachine>       // ActiveState is a toplevel state in StateMachine
  struct InternalState : sc::simple_state<InternalState, ActiveState>    // InternalState is a substate of ActiveState
  struct InternalState2 : sc::simple_state<InternalState2, ActiveState>  // InternalState2 is another substate of ActiveState
  ```
  — A SMACC2 state inherits from `SmaccState`, which is derived from `sc::simple_state`.
   </br>
   </br>


- State transitions are triggered by **events**: `struct MyEvent : sc::event<MyEvent> {};`</br>
  Events can be deferred or forwarded, see [chapter 5](https://github.com/CodeSports/State-Machine-Using-Boost-Statechart/tree/master/Chapter-5) and [chapter 6](https://github.com/CodeSports/State-Machine-Using-Boost-Statechart/tree/master/Chapter-6) of the eBook. </br>

  Event reactions (to specify in the `reactions` typedef of a state):</br>
  ```c++
  sc::transition< event, target_state > // without transition action
  sc::transition< event, target_state, common_context , transition_action > // with transition action
  sc::custom_reaction< event > // with react function  sc::result react( const event & );
  sc::deferral< event >  // deferred events are stored in a separate queue, which is emptied into the main queue when this state is exited.
  ```
  

  Custom event reaction, to use in the `react()` function for a `custom_reaction`:</br>
  ```c++
  return transit< state >(); // without transition action
  return transit< state >( transition_action, event ); // with transition action
  return terminate();
  return forward_event(); // forward event to the outer state
  return discard_event();
  ```
  See [this tutorial paragraph](https://www.boost.org/doc/libs/1_81_0/libs/statechart/doc/tutorial.html#TransitionActions) for more info on transition actions.

   </br>
- Note that the [Boost Statechart tutorials](https://www.boost.org/doc/libs/1_81_0/libs/statechart/doc/tutorial.html) discuss the use of multiple inheritance as a design pattern to retrieve information from the states:

  ```c++
  struct Running : IElapsedTime, sc::simple_state< Running, Active >
  ```
  
  The state `Running` is derived from `sc::simple_state` and from the interface class `IElapsedTime`. This interface class defines common methods to be implemented by each state.</br>
  </br>
  Similarly (though not necessarily exactly for the same reason):
  - A `smacc2::SmaccState` inherits from `sc::simple_state` and from `smacc2::ISmaccState`.</br>
  -  A `smacc2::SmaccStateMachineBase` inherits from `sc::asynchronous_state_machine` and from `smacc2::ISmaccStateMachine`.
   </br>
   </br>

- In Boost Statechart,  **orthogonal states** are states that have multiple active substates at once. </br>
In SMACC2 the word `orthogonal` is also used, but it has a very different meaning. See further down for more info on SMACC2 orthogonals.

</br>
</br>


## Asynchronous state machines

Given that the SMACC2 `SmaccStateMachineBase` is an `sc::asynchronous_state_machine`, and given that asynchronous state machines are not necessarily straighforward to understand upon a first read of the Boost Statechart [tutorials](https://www.boost.org/doc/libs/1_81_0/libs/statechart/doc/tutorial.html), this paragraph elaborates on this topic.
</br>
</br>

### What is the issue?

A **non-asynchronous** state machine (`sc::state_machine`) is instantiated, initiated, and then processes events, e.g.:

```c++
int main()
{
  StopWatch myWatch;
  myWatch.initiate();
  myWatch.process_event( EvStartStop() );
  myWatch.process_event( EvStartStop() );
  myWatch.process_event( EvReset() );
  return 0;
}
```

Each `process_event()` call takes a certain amount of time. This could be very little (e.g. processing an event that is simply discarded), but could also be significant (e.g. a multitude of state instances that need to be destructed/constructed upon a state change).

In above example, each `process_event()` is called consecutively, so it is impossible that a `process_event()` call is made while a previous one is still running.

In a multithreaded program however, with multiple sources of events, it could be possible that one thread's `process_event()` is preempted by another thread that also calls `process_event()`. This would obviously lead to erratic behavior.</br>
(Note that this is in fact irrespective of whether `process_events()` takes a long time or not, though obviously more likely in the former case.)</br>
 In other words: unless you'd implement your own solution, synchronous Boost Statechart state machines are not thread safe: it is not allowed to call `process_events()` from multiple threads.

</br>

As a side note: the Boost Statechart [tutorials](https://www.boost.org/doc/libs/1_81_0/libs/statechart/doc/tutorial.html) show a similar issue for the singlethreaded case, but this is not of particular relevance for now.

</br>

### So... asynchronous state machines?

The solution to above issue is to use an asynchronous state machine. The processing of an asynchronous state machine is split into:
- A `scheduler`, which receives events and stores them into a queue, and 
- A `processor`, which sequentially gets an event from the scheduler and processes it in the state machine.

Boost Statechart provides one type of scheduler: the `sc::fifo_scheduler`, which feeds the events to the processor in a 'First In First Out' fashion.</br>
Other types of schedulers are possible (e.g. priority- or deadline-based schedulers) but as of yet are not part of Boost Statechart.

The syntax is as follows:

```c++
int main()
{
  // Create a scheduler
  sc::fifo_scheduler<> my_scheduler( bool waitOnEmptyQueue );
  
  // Create a processor from the scheduler
  // This call instantiates the state machine
  auto my_processor_handle = my_scheduler.create_processor< state_machine_type >();
  
  // Initiate the processor
  my_scheduler.initiate_processor( my_processor_handle );
  
  // Optional: queue events
  //my_scheduler.queue_event( my_processor_handle, my_event1 );
  //my_scheduler.queue_event( my_processor_handle, my_event2 );
  
  // Run the scheduler by calling sc::fifo_scheduler::operator()
  my_scheduler();
}
``` 
Re. the `sc::fifo_scheduler` constructor:
- If `waitOnEmptyQueue` is `false`, then `sc::fifo_scheduler::operator()` returns when the queue is empty.
- If `waitOnEmptyQueue` is `true`, then `sc::fifo_scheduler::operator()` does not return on an empty queue, but waits for future events to be added (by calling `queue_event()` on the scheduler, e.g. from another thread or from the state machine itself),
  - In this case, a call of `fifo_scheduler<>::terminate()` is needed to interrupt the waiting and return from `sc::fifo_scheduler::operator()`.

</br>

For more examples of the use of asynchronous state machines, e.g. multiple processors from one scheduler, see the [Boost Statechart tutorials](https://www.boost.org/doc/libs/1_81_0/libs/statechart/doc/tutorial.html).

</br>
</br>

Let's now look at the SMACC2 `sm_atomic` example.


## The sm_atomic example

### Program startup

The state machine program starts with a call to `smacc2::run<state_machine_type>()`, see [sm_atomic_node.cpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/sm__atomic__node_8cpp.html):

```c++
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc2::run<sm_atomic::SmAtomic>();
}
```

`SmAtomic` is the state machine class.

`smacc2::run<T>()` is defined in [smacc_signal_detector.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__signal__detector_8hpp_source.html), see description below the code:

```c++
template <typename StateMachineType>
void run()
{
  // create the asynchronous state machine scheduler
  SmaccFifoScheduler scheduler1(true);
 
  // create the signalDetector component
  SignalDetector signalDetector(&scheduler1);
 
  // create the asynchronous state machine processor
  SmaccFifoScheduler::processor_handle sm =
    scheduler1.create_processor<StateMachineType>(&signalDetector);
 
  // initialize the asynchronous state machine processor
  signalDetector.setProcessorHandle(sm);
 
  scheduler1.initiate_processor(sm);
 
  //create a thread for the asynchronous state machine processor execution
  boost::thread otherThread(boost::bind(&sc::fifo_scheduler<>::operator(), &scheduler1, 0));
 
  // use the  main thread for the signal detector component (waiting actionclient requests)
  signalDetector.pollingLoop();
}
```
Let's have a look:

- First, a `SmaccFifoScheduler` is created.
  - A `SmaccFifoScheduler` is a typedef for an `sc::fifo_scheduler<SmaccFifoWorker, SmaccAllocator>`. (see [smacc_fifo_scheduler.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__fifo__scheduler_8hpp_source.html)) </br>
  - A `SmaccFifoWorker` is similarly an `sc::fifo_worker` and a `SmaccAllocator` an `std::allocator` (see [smacc_fifo_worker.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__fifo__worker_8hpp_source.html)).
  
  So this is more or less standard Boost Statechart, given `SmAtomic` is an `sc::asynchronous_state_machine`.

- Then a `SignalDetector` is created. This is a SMACC2 class.</br>
  We will check out its functionality later on, but for now notice that a few lines below a call is made to `signalDetector.pollingLoop()` and that the comments state that this loop waits for actionclient requests.

- The rest of the code is also standard for Boost Statechart asynchronous state machines:
  - Recall that the instantiation of the state machine class (i.e. SmAtomic) is done by the `fifo_scheduler<>::create_processor<>( args )` call and that the `args` are passed to the state machine constructor,
  - The `FifoScheduler` runs in a separate thread (`sc::fifo_scheduler<>::operator()`).

</br>

### smacc2::SignalDetector

So let's further look at the SignalDetector code ([smacc_signal_detector.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__signal__detector_8hpp_source.html) and [signal_detector.cpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/signal__detector_8cpp_source.html)).

In above run() function, calls were made to the SignalDetector constructor, `setProcessorHandle` and `pollingLoop` methods.</br>
- The constructor only sets some member variables, among others `initialized_ = false`.</br>
- `SignalDetector::setProcessorHandle` also just sets a member variable pointing to the handle.</br>
- `SignalDetector::pollingLoop` polls `initialized_` until it becomes true.

So the initialization is probably initiated from the state machine code, since the signalDetector was passed as an argument to `create_processor<>()` (and hence the state machine constructor).

Indeed, in the `ISmaccStateMachine` constructor ([smacc_state_machine.cpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__state__machine_8cpp_source.html)):
- A ROS2 Node is instantiated (automatically named corresponding to the state machine),
- `SignalDetector::initialize()` is called,
- `run_mode` parameter of the node is read and runMode set accordingly (`DEBUG` or `RELEASE`).

`SignalDetector::initialize()`:
- Calls `findUpdatableClientsAndComponents()` (more on this below, not relevant for now)
- Declares a ROS2 parameter `signal_detector_loop_freq`, and sets its loop frequency accordingly (standard: 20Hz),
- Sets `initialized_` to true.


</br>

Given that `initialized_` is true, `SignalDetector::pollingLoop` continues.</br>
We see that it polls *something* and spins the node:
  ```c++
  while (rclcpp::ok() && !end_)
  {
    pollOnce();
    rclcpp::spin_some(nh);
    r.sleep();
  }
  ```
  </br>

> 
> </br>
> 
> ### Intermediate conclusions #1
> 
> - A `FifoScheduler` and `Processor` are created to run an asynchronous state machine.
> - The state machine is created by `create_processor<>()`. The state machine constructor mainly
>   - Creates a **ROS2 node**, and
>   - Initializes the **SignalDetector**,
> - The `FifoScheduler` runs in a **separate thread**,
> - The `SignalDetector` polls *something* and spins the state machine node:
>   - At a configurable rate (default 20Hz),
>   - In the context of the **main thread**. </br>
> </br>
> 

 </br>

</br>

### The SignalDetector poll routine

`SignalDetector::pollOnce()` is implemented in [signal_detector.cpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/signal__detector_8cpp_source.html).

</br>
It does the following (see below for further clarification):

</br>

- Lock the state machine mutex,</br>
- Save the "current state counter" and "current state" of the state machine,
- Call `findUpdatableClientsAndComponents()`. This rebuilds a `std::vector<ISmaccUpdatable*>` **`updatableClients_`**,
- Call `executeUpdate(state_machine_node)` on each of the `ISmaccUpdatable`'s in `updatableClients_`,
- Then, IF the state machine is NOT `TRANSITIONING`, `CONFIGURING`, `ENTERING` or `EXITING`:
  - If the state has changed (current state counter != lastState_):
    - Call `findUpdatableStateElements(currentstate)`. This rebuilds a `std::vector<ISmaccUpdatable *>` **`updatableStateElements_`**
  - Call `executeUpdate(state_machine_node)` on each of the `ISmaccUpdatable`'s in `updatableStateElements_`,
- Call `rclcpp::spin_some(nh)`.</br>

</br>

[ QUESTION:&emsp; *The mutex is not related to Boost Statechart, so what is holding the FifoScheduler (in the other thread) from processing a state change event while `pollOnce` is executed (by the main thread)?* ]</br>
--> ANSWER: It seems that this mutex is also locked in `SmaccState::exit()` which blocks the transition. However: by then `specificNamedOnExit` and `notifyTransition` have been called?

[ QUESTION:&emsp; _Why is `spin_some` called twice: both here in `pollOnce()` as well as in `pollingLoop()`?_ ]


</br>
</br>

>
></br>
> 
>### Intermezzo
>
> The next paragraph introduces orthogonals, clients, client behafiors, etc.
> 
> From the current (27 jan 2023) [SMACC2 documentation](https://smacc.dev/intro-to-substate-objects/) we gather:
> - Orthogonals are classes:
>   - Instantiated in the state machine,
>   - Persistent through the lifetime of the **state machine**,
>   - That serve as a container for clients, client behaviors, orthogonal components,
> - Clients:
>   - Are also persistent for the life of the **state machine**,
>   - Are typically used to do things, like manage connections to outside nodes and devices,
>   - Contain code that we would want executed regardless of the current state,
>   - Are an important source of events.
> - Client behaviors:
>   - Are persistent for the life of the **state**,
>   - Are used to execute state specific behaviors.
>   - "In a given state, there can be multiple client behaviors in any orthogonal."</br>
>
> This is a rather concise explanation, but for now just accept that these concepts exist and that we will have to fill in the details later.
>
></br>
</br>
</br>

### findUpdatableClientsAndComponents()

Implemented in [signal_detector.cpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/signal__detector_8cpp_source.html). It:

- Clears `std::vector<ISmaccUpdatable*> updatableClients_`,
- Iterates over all **orthogonals** (`smacc2::ISmaccOrthogonal`) of the **state machine**,
- Iterates over all **clients** (`smacc2::ISmaccClient`) of each of the **orthogonals**,
- Checks which of these clients and which **components** of these clients (`smacc2::ISmaccComponent`) are derived from **`smacc2::ISmaccUpdatable`** (by using `dynamic_cast<>`),
- If they are an `smacc2::ISmaccUpdatable` (i.e. the `dynamic_cast` is successful), than the resulting pointer is added to `updatableClients_`.

</br>

In other words:
- On each signal detector poll loop, `updatableClients_` is rebuilt to hold:
  - All clients of each orthogonal of the state machine, and
  - All components of those clients,</br>
- If these are `ISmaccUpdatable`.

[ QUESTION:&emsp;_Given that orthogonals and clients are persistent throughout the state machine lifetime, why is this vector rebuilt on each poll? Maybe the client's components are not persistent? What are client components anyway?_]


</br>

### findUpdatableStateElements(currentstate)


Implemented in [signal_detector.cpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/signal__detector_8cpp_source.html). Similar to `findupdatableClientsAndComponents()`, it:

  - Clears `std::vector<ISmaccUpdatable*> updatableStateElements_`,
  - Iterates over all **orthogonals** (`smacc2::ISmaccOrthogonal`) of the **state machine**,
  - Adds all **clients behaviors** (`smacc2::ISmaccClientBehavior`) of each of the **orthogonals** to `updatableStateElements_` if they inherit from `ISmaccUpdatable`,
  - Adds the **current state** itself to `updatableStateElements_` if it inherits from `ISmaccUpdatable`,
  - Adds all **state reactors** (`smacc2::StateReactor`) of the **state** if they inherit from `ISmaccUpdatable`,
  - Adds all **event generators** (`smacc2::SmaccEventGenerator`) of the **state** if they inherit from `ISmaccUpdatable`.

  </br>

In other words:
  - After each state change, `updatableStateElements_` is rebuilt to hold:
    - Client behaviors of each orthogonal of the state machine,
    - The current state,
    - State reactors of the current state, and
    - Event generators of the current state,
  - if these are `ISmaccUpdatable`. 

Note that, although client behaviors are in the orthogonal (persistent to the state machine lifetime), they are added and removed from the orthogonal on each state change (i.e. client behaviors are persistent to the lifetime of the **state**).

</br>



### ISmaccUpdatable

Let's now look at the `executeUpdate()` calls. </br>
From the implementation in [smacc_updatable.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__updatable_8hpp_source.html) and [smacc_updatable.cpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__updatable_8cpp_source.html), we learn that:</br>

- The `ISmaccUpdatable` imposes the implementation of an `update()` method in the derived class,
- An `ISmaccUpdatable` optionally has a period (rclcpp::Duration),
- The `update()` method is called:
  - If no duration is set: each time when `execucuteUpdate(node)` is called (by the SignalDetector),
  - If a duration is set: when `execucuteUpdate(node)` is called AND the duration has passed since the previous `update()`.

In other words: an `ISmaccUpdatable` has its `update()` method called periodically, with period equal to, or a multiple of, the SignalDetector polling loop period.

</br>
</br>

> 
> </br>
> 
> ### Intermediate conclusions #2
> 
> - The **main thread** runs the SignalDetector **polling loop** and **spins the state machine node**,
> - This at a default frequency of 20Hz or according to a ROS2 parameter,
> - A state machine has **orthogonals**, with lifetime of the **state machine**,
> - An orthogonal has **clients** and clients have **client components**, with lifetime of the **state machine**,
> - An orthogonal has **client behaviors**, according to the **current state**,
> - A state has **state reactors** and **event generators**,
> - Clients, client components, client behaviors, states, state reactors and event generators **can be an `ISmaccUpdatable`**,</br>
> - `ISmaccUpdatables` have their **`update()` method periodically called** by the polling loop (i.e. the main thread),
> - This either at the polling loop frequency or at the set update period of the `ISmaccUpdatable`.
>
> </br>
> 

</br></br>

> 
> </br>
> 
> ### CAVEAT !
> 
> `dynamic_cast<>()` is used to check if the object (client, client behavior, etc) inherits from `ISmaccUpdatable`.</br> 
> However, `dynamic_cast<>()` returns `nullptr` in case of **private** inheritance.
>
> E.g. consider following class definitions:
> ```c++
> class MyClient1 : public smacc2::ISmaccClient, smacc2::ISmaccUpdatable {};
> class MyClient2 : public smacc2::ISmaccClient, public smacc2::ISmaccUpdatable {};
> ```
>
> </br>
>
> - `MyClient1` is **not** recognized as an `ISmaccUpdatable` and its `update()` is **not** called.
> - `MyClient2` works as expected.
> 
> </br>
> 




</br>
</br>

## The sm_atomic example, revisited

As it turns out, none of the clients, client behaviors, states, etc in the sm_atomic example are `ISmaccUpdatable`.</br>
So that's a bit of an anticlimax isn´t it?
 

Yet, we are better prepared now to dig through the sm_atomic code:

### The state machine

[sm_atomic.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/sm__atomic_8hpp_source.html)

Has a method `onInitialize()` that creates an orthogonal (class `OrTimer`).

</br>

### The orthogonal

[or_timer.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/sm__atomic_2include_2sm__atomic_2orthogonals_2or__timer_8hpp_source.html)

Has a method `onInitialize()` that creates a client (`ClRosTimer`) in itself (i.e. the orthogonal).

</br>

### The states

[st_state_1.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc2__sm__reference__library_2sm__atomic_2include_2sm__atomic_2states_2st__state__1_8hpp_source.html) has methods:
- `staticConfigure()`
- `runtimeConfigure()`
- `onEntry()`
- `onExit()`

`staticConfigure()` configures two client behaviors (`CbTimerCountdownLoop` and `CbTimerCountdownOnce`) in the orthogonal.

</br>

The event reaction is defined similar to Boost Statechart reactions, but with a somewhat different syntax:

```c++
  // Boost Statechart
  transition< eventtype, targetstate >
  
  // SMACC2
  Transition<                                 // Note the capital 'T'
    EvTimer<CbTimerCountdownOnce, OrTimer>,   // Event type
    State2,                                   // Target state
    SUCCESS                                   // Tag (see below)
    >
```
Note that, while Boost Statechart states have an `exit()` member function which is called just before the state object is destructed, they do not have the `onEntry()`, `onExit(),` and `..Configure()` member functions. More on that below.

</br>

[st_state_2.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc2__sm__reference__library_2sm__atomic_2include_2sm__atomic_2states_2st__state__2_8hpp_source.html)

Identical to `st_state_1`, except that `staticConfigure()` configures only one client behavior in the orthogonal.

</br>

### Transitions

Let's dig into the transitions first.

We find the implementation in [smacc_transition.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__transition_8hpp_source.html).

```c++
template <
  class Event, class Destination, typename Tag,
  class TransitionContext, void (TransitionContext::*pTransitionAction)(const Event &)>
class Transition
{
  ...
};
```

Now, this is rather strange at first, as there's five template parameters, whereas the example above took only three. </br>
The trick is that there's a forward declaration of `Transition` hidden at the bottom of [smacc_types.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__types_8hpp_source.html) which has the default values:

```c++
template <
  class Event, class Destination, typename Tag = default_transition_tags::default_transition_name,
  class TransitionContext = boost::statechart::detail::no_context<Event>,
  void (TransitionContext::*pTransitionAction)(const Event &) = &boost::statechart::detail::no_context<Event>::no_function>
class Transition;
```

The transition tags are also defined in that file:

```c++
// you can also use these other labels in order to have
// a better code readability and also to improve the visual representation
// in the viewer
struct DEFAULT {};
struct ABORT {};
struct SUCCESS {};
struct CANCEL {};
// struct PREEMPT {};
// struct REJECT {};
struct CONTINUELOOP {};
struct ENDLOOP {};
struct default_transition_name : SUCCESS {};
```

Comparing [smacc_transition.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__transition_8hpp_source.html) to the original Boost Statechart one (`/usr/include/boost/statechart/transition.hpp`), we see that both implement the `react_without_action()` and `react_with_action()` member functions, but the SMACC2 version has some extra functionality:

- It adds some log info (RCLCPP_DEBUG),
- It calls `specificNamedOnExit(state, tag_type)`,
- It calls `notifyTransition()` on the state

</br>

The implementation of `specificNamedOnExit(state,tag_type)` is found in [state_traits.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/state__traits_8hpp_source.html).</br>
It's rather tough code to read, so I did not fully validate this, but I'm pretty sure it does the following:
- Check if the state that is being exited has a specific `onExit(tag_type)` defined for the tag type that was specified in the Transition,
- And, if so: call that specific `onExit(tag_type)` on the exiting state.

</br>


Now we're at it, that same file also implements `standardOnExit(state)`. Similarly, this one seems to check if the state implements `onExit()` and calls it accordingly. `standardOnExit(state)` --and hence the state's `onExit()`-- is called from the standard Boost Statechart `exit()` call of the state (implementation in [`Smacc2::SmaccState::exit()`](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__state__base_8hpp_source.html) )

</br>
</br>

The implementation  of `notifyTransision()` is in [smacc_state_impl.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__state__impl_8hpp_source.html).</br>
It forwards the notification to `ISmaccState::notifyTransitionFromTransitionTypeInfo(TypeInfo::Ptr & transitionType)`. Reading through this it seems that:
- A `smacc2::ISmaccState` has a `smacc2::introspection::SmaccStateInfo`,
- A `SmaccStateInfo` has an `std::vector< SmaccTransitionInfo > transitions_`,
- The current transition's type (`TypeInfo`) is compared to all `SmaccTransitionInfo` structs in `transitions_`,
- If it is found, it is published on `/smacc/transition_log` as a `smacc2_msgs::msg::SmaccTransitionLogEntry`,
- If not, debug info is logged as RCLCPP_ERROR_STREAM.


[ QUESTION:&emsp; *in `ISmaccStateMachine::publishTransition()`: &emsp; `this->transitionLogHistory_.push_back(transitionLogEntry);` ---> This an evergrowing vector? There's no clear anywhere?*]

</br>

> 
> </br>
> 
> ### Intermediate conclusions #3
> 
> - SMACC2 Transitions (with captial 'T') optionally take an extra 'TAG' template parameter, 
> - States can implement specific `onExit(tag_type specific_tag)` member functions,
>   - These are called in case of a Transition that has that specific tag type,
> - States can also have a generic `onExit()` without tag type,
>   - This is called from the `exit()` function of the state (as it is an `sc::simple_state`),
> - Hence the specific `onExit(tag_type)` is called before the generic `onExit()` if both are present.
> - Transitions are published on `/smacc/transition_log` and a history of all these transition messages is kept in an (evergrowing?) vector.
>
>
> </br>
> 
> [ QUESTION:&emsp;*Does this imply that it is not intended to use `sc::custom_reaction< event >` and corresponding `sc::result react( const event & )` functions?* ]
>
> </br>
> 


</br>
</br>


### The Client

[cl_ros_timer.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/include_2ros__timer__client_2cl__ros__timer_8hpp_source.html) and [timer_client.cpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/timer__client_8cpp_source.html)

In the header file, the event is also declared:

```c++
template <typename TSource, typename TOrthogonal>
struct EvTimer : sc::event<EvTimer<TSource, TOrthogonal>> {};
```

[ QUESTION &emsp;*Not clear why EvTimer is a template class, afaik it could also be the following (less confusing for first time users):&emsp; 
`struct EvTimer : sc::event<EvTimer> {}; `*]</br>
[ POSSIBLE ANSWER: *I think the template definition is used to define source- and orthogonal-specific events. I.e. one type of client can be added to multiple orthogonals and send client- and orthogonal-specific events (as long as the orthogonals are of a different type).*]

</br>

The timer client seems to:
- Create a ROS2 timer in its `onInitialize()`,
- Its timer callback runs either one time (`bool oneshot = true`) or periodically (`oneshot = false`, default),
- The timer callback calls:
  - If it has been set: `this->onTimerTick_()`
    - `onTimerTick_` is set by a call to `ClRosTimer::onTimerTick(void (T::*callback)(), T * object)`),
  - and `postTimerEvent_()`
    - `postTimerEvent_` is set in `ClRosTimer::onOrthogonalAllocation()`
    - It resolves to `this->postEvent()`</br>
    [QUESTION:&emsp;~~is this general or client-specific?~~ It seems to be client-specific, e.g. it could as well call `postEvent(event)` instead, if an instantiated event object should be passed on]

</br>

Note that for the sm_atomic example, the timer client created by the orthogonal uses default parameter `oneshot = false`, i.e. it is a periodic timer client:</br>
[or_timer.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/sm__atomic_2include_2sm__atomic_2orthogonals_2or__timer_8hpp_source.html)
```c++
void onInitialize() override { auto client = this->createClient<cl_ros_timer::ClRosTimer>(1s); }
```

</br>
</br>

The call to `this->postEvent()` needs further study: </br>
It is the base class method `ISmaccState::postEvent()`, which resolves to `statemachine->postEvent()`, implemented in [smacc_state_machine_impl.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__state__machine__impl_8hpp_source.html).

These state machine `postEvent()` methods take an `EventLifeTime` which defaults to `ABSOLUTE` :
```c++
  template <typename EventType>
  void postEvent(EventType * ev, EventLifeTime evlifetime = EventLifeTime::ABSOLUTE);
 
  template <typename EventType>
  void postEvent(EventLifeTime evlifetime = EventLifeTime::ABSOLUTE);
```

</br>

`ISmaccStateMachine::postEvent()` instantiates a new event object, of type `EventType`, and then calls `ISmaccStateMachine::postEvent(event)`. </br>
 
&emsp;&emsp;&emsp;[ QUESTION:&emsp; ~~Ugly raw pointer. What is the lifetime of an event? Who deletes it?~~&emsp;See below:&emsp;`SignalDetector::postEvent(EventType* ev)` ] </br></br>

`ISmaccStateMachine::postEvent( event, eventLifeTime)`:
- Discards the event if its EventLifeTime is `CURRENT_STATE` and the current state is `EXITING` or `TRANSITIONING`, or else
- Propagates the event to the state reactors:
  - This calls `notifyEvent(event)` on each state reactor of the **state**, and recursively of all **parent states**,
- And finally calls `postEvent(event)` on the SignalDetector.

</br>

`SignalDetector::postEvent(EventType* ev)`:
- Converts the raw event pointer to an intrusive_ptr:
  - `boost::intrusive_ptr<EventType> weakPtrEvent = ev;`</br>
  - If I understand it correctly, this transfers ownership of the raw pointer to the intrusive_ptr, which will delete the pointer when the intrusive_ptr goes out of scope.</br>
  - &emsp;[QUESTION:&emsp;Is this correct?]</br>
  - This means that weakPtrEvent is somewhat of an ill-chosen name and should rather be sharedPtrEvent or intrusivePtrEvent.</br>
-  And then dispatches the event to the state machine **FifoScheduler queue**.
  

</br></br>

### The Client Behaviors


[`sm_atomic::State1`](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc2__sm__reference__library_2sm__atomic_2include_2sm__atomic_2states_2st__state__1_8hpp_source.html) 
and
[`sm_atomic::State2`](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc2__sm__reference__library_2sm__atomic_2include_2sm__atomic_2states_2st__state__2_8hpp_source.html)
configure client behaviors into the orthogonal in their `staticConfigure()` methods:

```c++
static void sm_atomic::State1::staticConfigure()
{
  configure_orthogonal<OrTimer, CbTimerCountdownLoop>(3);  // EvTimer triggers each 3 client ticks
  configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);  // EvTimer triggers once at 10 client ticks    //5?
}

static void sm_atomic::State2::staticConfigure()
{
  configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);  // EvTimer triggers once at 10 client ticks    //5?
}
```

</br>

Let's look at `configure_orthogonal<>(Args)` first, see [smacc_state_base.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__state__base_8hpp_source.html#l00039):


`configure_orthogonal()`  passes following lambda function to `configure_orthogonal_internal()`:</br>
&emsp;&emsp;`[=](ISmaccState * state) { state->configure<TOrthogonal, TBehavior>(args...); });`

</br>

`configure_orthogonal_internal()`:
- Populates a `ClientBehaviorInfoEntry` with:
  - Above lambda function pointer (`factoryFunction`),
  - The behavior typeid
  - The orthogonal typeid
- It adds a new entry to ab std::map of pairs ( `state`, `std::vector<ClientBehaviorInfoEntry>` ) if no entry exists yet for this state, and
- It adds the `ClientBehaviorInfoEntry` to the std::vector of the state.

[ _The factoryFunction is called by `entryStateInternal()`_ ]

</br>

Even though it is not being called yet, let's have a look at `state->configure<TOrthogonal, TBehavior>(args...);`

[smacc_state_impl.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__state__impl_8hpp_source.html)

- It get's the orthogonal from the state machine,
- It creates a shared_ptr to a new client behavior,
- It adds it to the orthogonal,
- It calls `onOrthogonalAllocation()` on the client behavior object.


</br></br>

Ok, let's look at the client behavior code now:

[cb_timer_countdown_loop.hpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/include_2ros__timer__client_2client__behaviors_2cb__timer__countdown__loop_8hpp_source.html)
and
[cb_timer_countdown_loop.cpp](https://robosoft-ai.github.io/smacc2_doxygen/master/html/cb__timer__countdown__loop_8cpp_source.html)

The assumed chronologic order is as follows:
- At some point, `entryStateInternal()` is called on the state, which executes the `state->configure<>(args...)`,
- This creates the client behavior; its constructor is called,
  - (For the `cb_timer_*` client behaviors, the constructor only initializes some member variables)
- Then the client behaviors are added to the orthogonal and their `onOrthogonalAllocation()` is called,
  - This defines a function that posts an event,
- At some point, the client behavior's `onEntry()` method is called. This:
  - Requests the client, which will
    - Return a pointer to the client from this orthogonal, or if not found,
    - Return a pointer to that client in another orthogonal, or if not found,
    - Return a nullpointer (which will crash if dereferenced),
  - Registers a callback function with the client, which is called by the client (through a `boost::signals2::connection`, which is not necessarily relevant, except that this implies that multiple callback functions (of multiple client behaviors) can be registered, which will be called consecutively),
  - The callback signal connection is done through the state machine, which ensures that these are disconnected upon state change. It seems that currentlly, states, state reactors and client behaviors can register such signals, see [`createSignalConnection()`](https://robosoft-ai.github.io/smacc2_doxygen/master/html/smacc__state__machine__impl_8hpp_source.html#l00355).



</br>
</br>

</br>

### Intermediate conclusions #4

Clients can implement following functions, which are chronologically called:
```c++
- Constructor
- template <typename TOrthogonal, typename TSourceObject> void onOrthogonalAllocation()
- void onInitialize()
```


Clients Behaviors similarly have:
```c++
- Constructor
- template <typename TOrthogonal, typename TSourceObject> void onOrthogonalAllocation()
- void onEntry()
- void onExit()
```


Other relevant functions of the `ISmaccClient` base class:
```c++
- void connectSignal(TSmaccSignal &signal, void(T::*callback)(), T *object)
- void postEvent()
- void postEvent(const EventType &ev)
```



