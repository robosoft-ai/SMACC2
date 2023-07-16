^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sm_dance_bot_warehouse_3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.16 (2023-07-16)
-------------------
* Merge branch 'humble' of https://github.com/robosoft-ai/SMACC2 into humble
* Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  * Attempt to fix weird issue with ros buildfarm
  * More on this buildfarm issue
  ---------
  Co-authored-by: brettpac <brettpac@pop-os.localdomain>
* Contributors: brettpac, pabloinigoblasco

2.3.6 (2023-03-12)
------------------

1.22.1 (2022-11-09)
-------------------
* pre-release
* Contributors: pabloinigoblasco

* removing husky demo
* publisher
* Feature/warehouse 3 improvements (#313)
  * improvements in navigation client behaviors and husky barrel demo
  * many improvements in action client and cb sequence for hysky barrel search
  * more and better navigation behaviors on husky barrel search demo
  * functionality improvements in navigation and improvements of warehouse 3, format
  * functionality improvements in navigation and improvements of warehouse 3 and husky
  * format
  * warehouse 3 improvements
  * merge galactic
  * merge fix
  * minor
* improvements in navigation client behaviors and husky barrel demo (#311)
  * improvements in navigation client behaviors and husky barrel demo
  * many improvements in action client and cb sequence for hysky barrel search
  * more and better navigation behaviors on husky barrel search demo
  * functionality improvements in navigation and improvements of warehouse 3, format
  * functionality improvements in navigation and improvements of warehouse 3 and husky
  * format
* husky_improvements (#299)
  * husky_improvements
  * different planners profiles for navigation
  * getting changes from galactic
  * planner switcher
  * using galactic branch files
  * fixing breaking changes
  * minor fix
  * removing nav from source files
  * merge
* Feature/galactic rolling merge (#288)
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * Update cb_navigate_global_position.hpp
  * Merging code from backport foxy and updates about autoware (#208)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * Add mergify rules file.
  * Try fixing CI for rolling. (#209)
  Merging to get backport working.
  * Remove example things from Foxy CI setup. (#214)
  * Add Autoware Auto Msgs into not-released dependencies. (#220)
  * Fix rolling builds (#222)
  * do not merge yet - Feature/odom tracker improvements and retry motion (#223)
  * odom tracker improvements
  * adding forward behavior retry funcionality
  * removing warnings (#213)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * dockerfiles (#225)
  * Fix code generators (#221)
  * Fix other build issues.
  * Update SM template and make example code clearly visible.
  * Remove use of node in the sm performance template.
  * Updated templated to use Blackboard storage.
  * Update template to resolve the global data correctly.
  * Update sm_name.hpp
  Co-authored-by: Pablo Iñigo Blasco <pablo@ibrobotics.com>
  * Feature/retry behavior warehouse 1 (#226)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * Update file for fake hardware simulation and add file for gazebo simulation.
  * docker build files for all versions
  * retry behavior warehouse 1
  * missing file
  * minor format fix
  * other minor changes
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * fix broken source build (#227)
  * Only rolling version should be pre-released on on master. (#230)
  * Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
  * Update file for fake hardware simulation and add file for gazebo simulation. (#224)
  * Update file for fake hardware simulation and add file for gazebo simulation.
  * Add ignition file and update repos files.
  * Feature/improvements warehouse3 (#228)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * docker build files for all versions
  * barrel demo
  * barrel search build fix and warehouse3
  * fixing startup problems in warehouse 3
  * fixing format and minor
  * minor
  * progress in barrel husky
  * minor
  * barrel demo
  * progress
  * fixing broken build
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * Feature/barrel - do not merge yet (#233)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * docker build files for all versions
  * barrel demo
  * barrel search build fix and warehouse3
  * fixing startup problems in warehouse 3
  * fixing format and minor
  * minor
  * progress in barrel husky
  * minor
  * barrel demo
  * minor
  * barrel search updates
  * making models local
  * red picuup
  * multiple controllable leds plugin
  * progress in husky demo
  * progressing in husky demo
  * improving navigation behaviors
  * more merge
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * Feature/docker improvements march 2022 (#235)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * docker build files for all versions
  * barrel demo
  * barrel search build fix and warehouse3
  * fixing startup problems in warehouse 3
  * fixing format and minor
  * minor
  * progress in barrel husky
  * minor
  * barrel demo
  * minor
  * barrel search updates
  * making models local
  * red picuup
  * multiple controllable leds plugin
  * progress in husky demo
  * progressing in husky demo
  * improving navigation behaviors
  * more merge
  * docker improvements
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * Use correct upstream .repos files for source builds (#243)
  * Correct mergify branch names (#246)
  * Correct name of source-build job and bump version of action (#242) (#247)
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Pablo Iñigo Blasco <pablo@ibrobotics.com>
  * Update galactic source build job name (#250)
  * Galactic source build: update .repos file, bump action version and use correct version of upstream packages (backport #241) (#248)
  Co-authored-by: Denis Štogl <denis@stogl.de>
  * fixing rolling build (#239)
  * fixing rolling build
  * trying to fix dependencies
  * missing repo
  * fixing to focal by the moment
  * more fixing rolling build
  * minor
  * cache matrix rolling and source build package
  * minor
  * minor
  * missing repo
  * missing deps
  * fixing building issue
  * typo
  * fixing broken build
  * build fix
  * restoring workflow files (#252)
  * restoring files (#253)
  * Fix checkout branches for scheduled builds (#254)
  * correct checkout branch on scheduled build
  * Update foxy-source-build.yml
  * Feature/fixing husky build rolling (#257)
  * restoring files
  * making husky project build on rolling
  * Feature/fixing husky build rolling (#258)
  * restoring files
  * making husky project build on rolling
  * husky progress
  * Update README.md (#262)
  * Feature/fixing ur demos (#261)
  * restoring files
  * fixes
  * Feature/fixing type string walker (#263)
  * restoring files
  * fixing type string walker threesome demo
  * Update README.md (#266)
  * Update README.md (#267)
  * Update README.md (#268)
  * Significant update in Getting Started Instructions (#269)
  * Significant update in Getting Started Instructions
  * Remove trailing spaces.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  * fixing ur demo (#273)
  * fix: initialise conditionFlag as false (#274)
  * precommit fix (#280)
  I merge this in red for focal-rolling because it was already broken anyway and it is only a minor update of the precommit
  * Ignore packages which should not be released.
  * Added changelogs.
  * 0.4.0
  * Revert "Ignore packages which should not be released."
  This reverts commit ee2cc86db3c0a24f9eb0a9e33217de3f7a691a1c.
  * Fix urls to index.ros.org (#284)
  * Fix foxy source build config to use repos file from foxy branch. (#285)
  * adding spawn entity delays
  * more on backport
  * more on backport
  * disappeared ur_msgs denis repo
  * fixing sm_dance_bot examples
  * working on fix of image messages for husky_barrel demo
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  Co-authored-by: mergify[bot] <37929162+mergify[bot]@users.noreply.github.com>
  Co-authored-by: brettpac <brettpac@users.noreply.github.com>
* Revert "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
* Contributors: Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco

0.3.0 (2022-04-04)
------------------

0.0.0 (2022-11-09)
------------------
* removing husky demo
* publisher
* Feature/warehouse 3 improvements (#313)
  * improvements in navigation client behaviors and husky barrel demo
  * many improvements in action client and cb sequence for hysky barrel search
  * more and better navigation behaviors on husky barrel search demo
  * functionality improvements in navigation and improvements of warehouse 3, format
  * functionality improvements in navigation and improvements of warehouse 3 and husky
  * format
  * warehouse 3 improvements
  * merge galactic
  * merge fix
  * minor
* improvements in navigation client behaviors and husky barrel demo (#311)
  * improvements in navigation client behaviors and husky barrel demo
  * many improvements in action client and cb sequence for hysky barrel search
  * more and better navigation behaviors on husky barrel search demo
  * functionality improvements in navigation and improvements of warehouse 3, format
  * functionality improvements in navigation and improvements of warehouse 3 and husky
  * format
* husky_improvements (#299)
  * husky_improvements
  * different planners profiles for navigation
  * getting changes from galactic
  * planner switcher
  * using galactic branch files
  * fixing breaking changes
  * minor fix
  * removing nav from source files
  * merge
* Feature/galactic rolling merge (#288)
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * Update cb_navigate_global_position.hpp
  * Merging code from backport foxy and updates about autoware (#208)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * Add mergify rules file.
  * Try fixing CI for rolling. (#209)
  Merging to get backport working.
  * Remove example things from Foxy CI setup. (#214)
  * Add Autoware Auto Msgs into not-released dependencies. (#220)
  * Fix rolling builds (#222)
  * do not merge yet - Feature/odom tracker improvements and retry motion (#223)
  * odom tracker improvements
  * adding forward behavior retry funcionality
  * removing warnings (#213)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * dockerfiles (#225)
  * Fix code generators (#221)
  * Fix other build issues.
  * Update SM template and make example code clearly visible.
  * Remove use of node in the sm performance template.
  * Updated templated to use Blackboard storage.
  * Update template to resolve the global data correctly.
  * Update sm_name.hpp
  Co-authored-by: Pablo Iñigo Blasco <pablo@ibrobotics.com>
  * Feature/retry behavior warehouse 1 (#226)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * Update file for fake hardware simulation and add file for gazebo simulation.
  * docker build files for all versions
  * retry behavior warehouse 1
  * missing file
  * minor format fix
  * other minor changes
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * fix broken source build (#227)
  * Only rolling version should be pre-released on on master. (#230)
  * Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
  * Update file for fake hardware simulation and add file for gazebo simulation. (#224)
  * Update file for fake hardware simulation and add file for gazebo simulation.
  * Add ignition file and update repos files.
  * Feature/improvements warehouse3 (#228)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * docker build files for all versions
  * barrel demo
  * barrel search build fix and warehouse3
  * fixing startup problems in warehouse 3
  * fixing format and minor
  * minor
  * progress in barrel husky
  * minor
  * barrel demo
  * progress
  * fixing broken build
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * Feature/barrel - do not merge yet (#233)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * docker build files for all versions
  * barrel demo
  * barrel search build fix and warehouse3
  * fixing startup problems in warehouse 3
  * fixing format and minor
  * minor
  * progress in barrel husky
  * minor
  * barrel demo
  * minor
  * barrel search updates
  * making models local
  * red picuup
  * multiple controllable leds plugin
  * progress in husky demo
  * progressing in husky demo
  * improving navigation behaviors
  * more merge
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * Feature/docker improvements march 2022 (#235)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * docker build files for all versions
  * barrel demo
  * barrel search build fix and warehouse3
  * fixing startup problems in warehouse 3
  * fixing format and minor
  * minor
  * progress in barrel husky
  * minor
  * barrel demo
  * minor
  * barrel search updates
  * making models local
  * red picuup
  * multiple controllable leds plugin
  * progress in husky demo
  * progressing in husky demo
  * improving navigation behaviors
  * more merge
  * docker improvements
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * Use correct upstream .repos files for source builds (#243)
  * Correct mergify branch names (#246)
  * Correct name of source-build job and bump version of action (#242) (#247)
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Pablo Iñigo Blasco <pablo@ibrobotics.com>
  * Update galactic source build job name (#250)
  * Galactic source build: update .repos file, bump action version and use correct version of upstream packages (backport #241) (#248)
  Co-authored-by: Denis Štogl <denis@stogl.de>
  * fixing rolling build (#239)
  * fixing rolling build
  * trying to fix dependencies
  * missing repo
  * fixing to focal by the moment
  * more fixing rolling build
  * minor
  * cache matrix rolling and source build package
  * minor
  * minor
  * missing repo
  * missing deps
  * fixing building issue
  * typo
  * fixing broken build
  * build fix
  * restoring workflow files (#252)
  * restoring files (#253)
  * Fix checkout branches for scheduled builds (#254)
  * correct checkout branch on scheduled build
  * Update foxy-source-build.yml
  * Feature/fixing husky build rolling (#257)
  * restoring files
  * making husky project build on rolling
  * Feature/fixing husky build rolling (#258)
  * restoring files
  * making husky project build on rolling
  * husky progress
  * Update README.md (#262)
  * Feature/fixing ur demos (#261)
  * restoring files
  * fixes
  * Feature/fixing type string walker (#263)
  * restoring files
  * fixing type string walker threesome demo
  * Update README.md (#266)
  * Update README.md (#267)
  * Update README.md (#268)
  * Significant update in Getting Started Instructions (#269)
  * Significant update in Getting Started Instructions
  * Remove trailing spaces.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  * fixing ur demo (#273)
  * fix: initialise conditionFlag as false (#274)
  * precommit fix (#280)
  I merge this in red for focal-rolling because it was already broken anyway and it is only a minor update of the precommit
  * Ignore packages which should not be released.
  * Added changelogs.
  * 0.4.0
  * Revert "Ignore packages which should not be released."
  This reverts commit ee2cc86db3c0a24f9eb0a9e33217de3f7a691a1c.
  * Fix urls to index.ros.org (#284)
  * Fix foxy source build config to use repos file from foxy branch. (#285)
  * adding spawn entity delays
  * more on backport
  * more on backport
  * disappeared ur_msgs denis repo
  * fixing sm_dance_bot examples
  * working on fix of image messages for husky_barrel demo
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  Co-authored-by: mergify[bot] <37929162+mergify[bot]@users.noreply.github.com>
  Co-authored-by: brettpac <brettpac@users.noreply.github.com>
* Revert "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
* Ignore packages which should not be released.
* Feature/master rolling to galactic backport (#236)
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * replanning for all our examples
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * Update cb_navigate_global_position.hpp
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * Merging code from backport foxy and updates about autoware (#208)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * Add mergify rules file.
  * Try fixing CI for rolling. (#209)
  Merging to get backport working.
  * some reordering fixes
  * Remove example things from Foxy CI setup. (#214)
  * Add Autoware Auto Msgs into not-released dependencies. (#220)
  * Fix rolling builds (#222)
  * do not merge yet - Feature/odom tracker improvements and retry motion (#223)
  * odom tracker improvements
  * adding forward behavior retry funcionality
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * removing warnings (#213)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * docker build files for all versions
  * dockerfiles (#225)
  * Fix code generators (#221)
  * Fix other build issues.
  * Update SM template and make example code clearly visible.
  * Remove use of node in the sm performance template.
  * Updated templated to use Blackboard storage.
  * Update template to resolve the global data correctly.
  * Update sm_name.hpp
  Co-authored-by: Pablo Iñigo Blasco <pablo@ibrobotics.com>
  * Feature/retry behavior warehouse 1 (#226)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * Update file for fake hardware simulation and add file for gazebo simulation.
  * docker build files for all versions
  * retry behavior warehouse 1
  * missing file
  * minor format fix
  * other minor changes
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * barrel demo
  * barrel search build fix and warehouse3
  * fixing startup problems in warehouse 3
  * fix broken source build (#227)
  * fixing format and minor
  * minor
  * progress in barrel husky
  * minor
  * Only rolling version should be pre-released on on master. (#230)
  * barrel demo
  * minor
  * barrel search updates
  * making models local
  * red picuup
  * Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
  * multiple controllable leds plugin
  * progress in husky demo
  * Update file for fake hardware simulation and add file for gazebo simulation. (#224)
  * Update file for fake hardware simulation and add file for gazebo simulation.
  * Add ignition file and update repos files.
  * progressing in husky demo
  * improving navigation behaviors
  * Feature/improvements warehouse3 (#228)
  * minor changes
  * replanning for all our examples
  * backport to foxy
  * minor format
  * minor linking errors foxy
  * Foxy backport (#206)
  * minor formatting fixes
  * Fix trailing spaces.
  * Correct codespell.
  * Correct python linters warnings.
  * Add galactic CI build because Navigation2 is broken in rolling.
  * Add partial changes for ament_cpplint.
  * Add tf2_ros as dependency to find include.
  * Disable ament_cpplint.
  * Disable some packages and update workflows.
  * Bump ccache version.
  * Ignore further packages
  * Satisfy ament_lint_cmake
  * Add missing licences.
  * Disable cpplint and cppcheck linters.
  * Correct formatters.
  * branching example
  * Disable disabled packages
  * Update ci-build-source.yml
  * Change extension
  * Change extension of imports.
  * Enable cppcheck
  * Correct formatting of python file.
  * Included necessary package and edited Threesome launch
  Changed...
  ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
  Added:
  First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
  * Rename header files and correct format.
  * Add workflow for checking doc build.
  * Update doxygen-check-build.yml
  * Create doxygen-deploy.yml
  * Use manual deployment for now.
  * Create workflow for testing prerelease builds
  * Use docs/ as source folder for documentation
  * Use docs/ as output directory.
  * Rename to smacc2 and smacc2_msgs
  * Correct GitHub branch reference.
  * Update name of package and package.xml to pass liter.
  * Execute on master update
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  * Update description table.
  * Update table
  * Copy initial docs
  * Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * Opened new folder for additional tracing contents
  * Delete tracing directory
  * Moved tracing.md to tracing directory
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Created alternative ManualTracing
  * added new sm markdowns
  * added a dockerfile for Rolling and Galactic
  * Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * changed wording "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * reactivating smacc2 nav clients for rolling via submodules
  * renamed tracing events after
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * additional cleanup
  * cleanup
  * cleanup
  * edited tracing.md to reflect new tracing event names
  * Enable build of missing rolling repositories.
  * Enable Navigation2 for semi-binary build.
  * Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
  * updated mentions of SMACC/ROS to SMACC2/ROS2
  * some progress on navigation rolling
  * renamed folders, deleted tracing.md, edited README.md
  * added smacc2_performance_tools
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Update smacc2_rta command across readmes
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * minor formatting
  * Add galactic CI setup and rename rolling files. (#58)
  * Fix source CI and correct README overview. (#62)
  * Update c_cpp_properties.json
  * changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  also noticed a note I had made while producing these that was not removed
  * update doxygen links (#70)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme Updates (#72)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More Readme (#74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * created new sm from sm_respira_1 (#76)
  * Feature/core and navigation fixes (#78)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * Feature/aws demo progress (#80)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * sm_advanced_recovery_1 reworked (#83)
  * sm_advanced_recovery_1 reworked
  * fix pre-commit
  * Trying to fix Pre-Commit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_advanced_recovery_1 (#84)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * More sm_advanced_recovery_1 work (#85)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 round 4 (#86)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#87)
  * sm_atomic_performance_test_a_2
  * sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 (#90)
  * sm_multi_stage_1
  * fixing precommit
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more sm_multi_stage_1 (#91)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Update README.md
  updated launch command
  * Wait topic message client behavior (#81)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * attempting precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/wait nav2 nodes client behavior (#82)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * Correct all linters and formaters.
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * Feature/aws demo progress (#92)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * Feature/sm dance bot fixes (#93)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * Feature/sm aws warehouse (#94)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * merge and progress
  * fix format
  * Feature/sm dance bot fixes (#95)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * minor format
  * Remove some compile warnings. (#96)
  * Feature/cb pause slam (#98)
  * base for the sm_aws_aarehouse navigation
  * progressing in aws navigation
  * minor
  * several core improvements during navigation testing
  * formatting improvements
  * progress in aws navigation demo
  * format improvements
  * format improvements
  * more on navigation
  * new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  * formatting
  * adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
  * progress in aws navigation demo
  * minor format
  * navigation parameters fixes on sm_dance_bot
  * minor format
  * minor
  * formatting
  * cb pause slam client behavior
  * sm_dance_bot_lite (#99)
  * sm_dance_bot_lite
  * precommit
  * Updates yaml
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Rename doxygen deployment workflow (#100)
  * minor hotfix
  * sm_dance_bot visualizing turtlebot3 (#101)
  * Feature/dance bot launch gz lidar choice (#102)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * Feature/sm dance bot lite gazebo fixes (#104)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * sm_multi_stage_1 doubling (#103)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot strikes back gazebo fixes (#105)
  * sm_dance_bot visualizing turtlebot3
  * cleaning and lidar show/hide option
  * cleaning files and making formatting work
  * more fixes
  * gazebo fixes, to show the robot and the lidar
  * format fixes
  * gazebo fixes for sm_dance_bot_strikes_back
  * precommit cleanup run (#106)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * aws demo (#108)
  * aws demo
  * format
  * got sm_multi_stage_1 working (barely) (#109)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#110)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Brettpac branch (#111)
  * got sm_multi_stage_1 working (barely)
  * gaining traction sm_multi_stage_1
  * more
  * don't remember
  * making progress
  * More
  * keep hammering
  * two stages
  * 3 part
  * 4th stage
  * 5th stage
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * a3 (#113)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Remove neo_simulation2 package. (#112)
  * Remove neo_simulation2 package.
  * Correct formatting.
  * Enable source build on PR for testing.
  * Adjust build packages of source CI
  * more sm_multi_stage_1 (#114)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * mm (#115)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * diverse improvements navigation and performance (#116)
  * diverse improvements navigation and performance
  * minor
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  * Feature/diverse improvemets navigation performance (#117)
  * diverse improvements navigation and performance
  * minor
  * additional linting and formatting
  * Remove merge markers from a python file. (#119)
  * Feature/slam toggle and smacc deep history (#122)
  * progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
  * going forward in testing sm_dance_bot introducing slam pausing/resuming funcionality
  * feature/more_sm_dance_bot_fixes
  * minor format
  * minor (#124)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * more changes in sm_dance_bot (#125)
  * Move method after the method it calls. Otherwise recursion could happen. (#126)
  * Feature/dance bot s pattern (#128)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * noticed typo
  Finnaly > Finally
  * Feature/dance bot s pattern (#129)
  * more changes in sm_dance_bot
  * polishing sm_dance_bot and s-pattern
  * more refinement in sm_dance_bot
  * First working version of sm template and template generator. (#127)
  * minor tweaks (#130)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm dance bot refine (#131)
  * more changes in sm_dance_bot
  * minor
  * Feature/sm dance bot refine 2 (#132)
  * more changes in sm_dance_bot
  * minor
  * build fix
  * waypoints navigator bug (#133)
  * minor tuning to mitigate overshot issue cases
  * progress in the sm_dance_bot tests (#135)
  * some more progress on markers cleanup
  * minor format issues (#134)
  * sm_dance_bot_lite (#136)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Resolve compile wanings (#137)
  * Add SM core test (#138)
  * minor navigation improvements (#141)
  * using local action msgs (#139)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * Feature/nav2z renaming (#144)
  * using local action msgs
  * removing sm_dance_bot_msgs
  * pending references
  * navigation 2 stack renaming
  * formatting
  * added SVGs to READMEs of atomic, dance_bot, and others (#140)
  * added remaining SVGs to READMEs (#145)
  * added remaining SVGs to READMEs
  * precommit cleanup
  * Update package list. (#142)
  * removing parameters smacc (#147)
  * removing parameters smacc
  * workflows update
  * workflow
  * Noticed launch command was incorrect in README.md
  fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
  * Fix CI: format fix python version (#148)
  * Add SM Atomic SM generator. (#143)
  * Remove node creation and create only a logger. (#149)
  * Rolling Docker environment to be executed from any environment (#154)
  * Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * slight waypoint 4 and iterations changes so robot can complete course (#155)
  * Feature/migration moveit client (#151)
  * initial migration to smacc2
  * fixing some errors introduced on formatting
  * missing dependency
  * fixing some more linting warnings
  * minor
  * removing test from main moveit cmake
  * test ur5
  * progressing in the moveit migration testing
  * updating format
  * adding .reps dependencies and also fixing some build errors
  * repos dependency
  * adding dependency to ur5 client
  * docker refactoring
  * minor
  * progress on move_it PR
  * minor dockerfile test workaround
  * improving dockerfile for building local tests
  * minor
  * fixing compiling issues
  * update readme (#164)
  * update readme
  * more readme updates
  * more
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * initial state machine transition timestamp (#165)
  * moved reference library SMs to smacc2_performance_tools (#166)
  * moved reference library SMs to smacc2_performance_tools
  * pre-commit cleanup
  * Add QOS durability to SmaccPublisherClient (#163)
  * feat: add qos durability to SmaccPublisherClient
  * fix: add a missing colon
  * refactor: remove line
  * feat: add reliability qos config
  * Feature/testing moveit behaviors (#167)
  * more testing on moveit
  * progress on moveit
  * more testing on moveit behaviors
  * minor configuration
  * fixing pipeline error
  * fixing broken master build
  * sm_pubsub_1 (#169)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_pubsub_1 part 2 (#170)
  * sm_pubsub_1 part 2
  * sm_pubsub_1 part 2
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_advanced_recovery_1 renaming (#171)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * sm_multi_stage_1 reworking (#172)
  * multistage modes
  * sm_multi_stage sequences
  * sm_multi_state_1 steps
  * sm_multi_stage_1 sequence d
  * sm_multi_stage_1 c sequence
  * mode_5_sequence_b
  * mode_4_sequence_b
  * sm_multi_stage_1 most
  * finishing touches 1
  * readme
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/aws navigation sm dance bot (#174)
  * repo dependency
  * husky launch file in sm_dance_bot
  * Add dependencies for husky simulation.
  * Fix formatting.
  * Update dependencies for husky in rolling and galactic.
  * minor
  * progress on aws navigation and some other refactorings on navigation clients and behaviors
  * more on aws demo
  * fixing broken build
  * minor
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  * minor changes (#175)
  * warehouse2 (#177)
  * Waypoint Inputs (#178)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * wharehouse2 progress (#179)
  * format (#180)
  * sm_dance_bot_warehouse_3 (#181)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/sm warehouse 2 13 dec 2 (#182)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * Brettpac branch (#184)
  * sm_dance_bot_warehouse_3
  * Redoing sm_dance_bot_warehouse_3 waypoints
  * More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * SrConditional fixes and formatting (#168)
  * fix: some formatting and templating on SrConditional
  * fix: move trigger logic into headers
  * fix: lint
  * Feature/wharehouse2 dec 14 (#185)
  * warehouse2
  * minor
  * Feature/sm warehouse 2 13 dec 2 (#186)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * finetuning waypoints (#187)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  * Feature/cb pure spinning (#188)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * Feature/cb pure spinning (#189)
  * format
  * more changes and headless
  * merge
  * headless and other fixes
  * default values
  * minor
  * pure spinning behavior missing files
  * minor changes (#190)
  * Feature/planner changes 16 12 (#191)
  * minor changes
  * more fixes
  * minor
  * minor
  * Feature/replanning 16 dec (#193)
  * minor changes
  * replanning for all our examples
  * several fixes (#194)
  * minor changes (#195)
  * Feature/undo motion 20 12 (#196)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * tuning warehouse3 (#197)
  * Feature/undo motion 20 12 (#198)
  * minor changes
  * replanning for all our examples
  * improving undo motion navigation warehouse2
  * minor
  * undo tuning and errors
  * format
  * Feature/sync 21 12 (#199)
  * minor changes
  * replanning for all our examples
  * format issues
  * Feature/warehouse2 22 12 (#200)
  * minor changes
  * replanning for all our examples
  * format issues
  * finishing warehouse2
  * Feature/warehouse2 23 12 (#201)
  * minor changes
  * replanning for all our examples
  * tuning and fixes (#202)
  * Feature/minor tune (#203)
  * tuning and fixes
  * minor tune
  * fixing warehouse 3 problems, and other core improvements (#204)
  * fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
  * weird moveit not downloaded repo
  * added missing file from warehouse2 (#205)
  * backport to foxy
  * minor format
  * minor linking errors foxy
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  * missing
  * missing sm
  * updating subscriber publisher components
  * progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
  * refining cp subscriber cp publisher
  * improvements in smacc core adding more components mostly developed for autoware demo
  * autoware demo
  * missing
  * foxy ci
  * fix
  * minor broken build
  * some reordering fixes
  * minor
  * docker files for different revisions, warnings removval and more testing on navigation
  * fixing docker for foxy and galactic
  * docker build files for all versions
  * barrel demo
  * barrel search build fix and warehouse3
  * fixing startup problems in warehouse 3
  * fixing format and minor
  * minor
  * progress in barrel husky
  * minor
  * barrel demo
  * progress
  * fixing broken build
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
  * more merge
  * docker improvements
  * master rolling to galactic backport
  * fixing build
  * testing dance bot demos
  * updating galactic repos
  * runtime dependency
  * restoring ur dependency
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
  Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
* Contributors: Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco
