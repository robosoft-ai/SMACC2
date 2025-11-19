^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sm_simple_action_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2025-11-19)
-------------------
* Complete self-contained demo implementation with 3-state workflow
  * Added StState3 success completion state with informative onEntry message
  * Created auto_mode_trigger.py script for automatic demo initialization
  * Implemented comprehensive Python launch file with logging infrastructure
  * Added Fibonacci action server to launch configuration
  * Updated state transitions: StState2 now transitions to StState3 on action success
  * Fixed auto-trigger shutdown sequence to prevent TypeError on exit
  * Optimized Fibonacci action order from 100 to 10 for faster demo completion
  * Added timestamped log directory creation with fallback error handling
  * Integrated konsole terminal support for multi-window process visualization
  * Removed outdated XML launch file (sm_simple_action_client.launch)
  * Comprehensive README documentation covering:
    - State machine architecture with 3-state flow diagram
    - Automated demo and manual control instructions
    - Monitoring commands and troubleshooting guide
    - Component reference for clients, behaviors, and orthogonals
    - Learning objectives for SMACC2 patterns
* Bug fixes:
  * Fixed config file path to use correct simple_action_client_example_config.yaml
  * Resolved rclpy shutdown race condition in auto_mode_trigger
  * Updated all documentation references from order=100 to order=10
* Contributors: Claude Code, brettpac

2.3.19 (2025-06-17)
-------------------
* simple state machine with an action client and a topic subscriber (`#543 <https://github.com/robosoft-ai/SMACC2/issues/543>`_)
  * create a simple state machine with two states, one action server client and one subscriber client
  * added the template copyright and license info to all code files
  * renamed package and files to fit the established naming convention
  * added missing license for mode_selection_client and orhtogonals files
  * moved license to the top of the cl_mode_select.hpp file
  * fixed formatting errors
  ---------
  Co-authored-by: Sukhraj Klair <sukhraj.k@radskunkworks.com>
* Contributors: sukhrajklair

2.4.1 (2023-07-06)
------------------

2.3.16 (2023-07-16)
-------------------

2.3.18 (2023-04-24)
-------------------

2.3.8 (2023-03-12 21:50)
------------------------

2.3.7 (2023-03-12 21:45)
------------------------

2.3.6 (2023-03-12 20:30)
------------------------

2.3.5 (2023-03-07 00:14)
------------------------

2.3.4 (2023-03-07 00:02)
------------------------

2.3.3 (2023-03-02 22:58)
------------------------

2.3.2 (2023-03-02 22:22)
------------------------

2.3.1 (2022-11-28)
------------------

1.22.1 (2022-11-09 20:22)
-------------------------

1.22.0 (2022-11-09 19:53)
-------------------------
