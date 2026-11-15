# PX4 test tools

`px4_run.sh` runs a PX4 multirotor reference machine against a headless PX4 SITL (Gazebo x500) and
a micro-ROS agent, and reports how the flight and the shutdown went. The machine binary is started
directly so its exit status is observed. Developer machine only, never on a pull request.

```bash
cd <workspace>
bash src/SMACC2/smacc2_performance_tools/px4_test_tools/px4_run.sh install /tmp/px4_1 \
  ~/PX4-Autopilot/build/px4_sitl_default
```

| Argument | |
|--|--|
| install prefix | holds the machine, `cl_px4_mr`, `px4_msgs` and `micro_ros_agent` |
| output directory | receives `sm.log`, `px4.log`, `agent.log` |
| PX4 build directory | `build/px4_sitl_default` of a PX4-Autopilot checkout, built with `make px4_sitl` |
| `tsan` or `asan`, overlay prefix | optional, see below |

| Environment | Default | |
|--|--|--|
| `PKG` | `sm_cl_px4_mr_test_1` | the machine to run |
| `END_STATE` | `StLanded` | the state at which the machine is stopped |
| `MAX_WAIT` | 420 | seconds to wait for that state |
| `OWN_AGENT` | 0 | 1 starts no agent, for a machine that launches its own (tests 2, 4, 5); the report then counts agent processes the machine left behind |

**Start QGroundControl first.** PX4 refuses to arm without a ground station connected, and the
machine then waits forever in its first state. The script warns when it does not see one. The run
uses ROS domain 0, the domain PX4's DDS client uses. Exit status 0 when the end state was reached and
the machine stopped without hanging.

## Reading the report

| Line | Meaning |
|--|--|
| `verdict` | whether the end state was reached, and after how long; `died` means the machine exited by itself, `timeout` or `stall` that it stopped progressing |
| `SM exit` | the machine's exit status after SIGINT and how long shutdown took; `SM HANG` means it was still alive 120 s later |
| `leftover after` | simulator or node processes still running when the script ended; should be 0 |
| `== states` | every state entered, in order |
| `== shutdown` | the shutdown markers the core logged, in order |
| `== ERROR ...` | counts of error and fatal log lines, discarded events, DDS teardown errors and crashes, then the most frequent error lines |

All logs are in the output directory: `sm.log` for the machine, one log per simulator process.

The extra `== px4` line counts PX4's own arming, takeoff and landing messages.

## Under a sanitizer

The simulator and its ROS packages stay uninstrumented; only the SMACC2 packages are rebuilt, into
their own build and install bases, as an overlay on the normal install:

```bash
cd <workspace> && source install/setup.bash
colcon build --build-base build_tsan --install-base install_tsan --packages-select smacc2 cl_px4_mr sm_cl_px4_mr_test_1 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread,undefined" -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=thread,undefined"
```

For ASan use `-fsanitize=address,undefined -fsanitize-recover=address` in the compile flags and
`-fsanitize=address,undefined` in the two linker flags, with `build_asan` and `install_asan`. Then
add `tsan install_tsan` or `asan install_asan` to the command line. The script prints which
`libsmacc2.so` the binary resolves to, runs it with the suppression files in `.github/scripts/`, and
ends with the report classifier's verdict (`.github/scripts/sanitizer_reports.py`). TSan needs
address-space randomization off, which the script does with `setarch -R`.
