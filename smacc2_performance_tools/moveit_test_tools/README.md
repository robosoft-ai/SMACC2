# MoveIt test tools

`panda_run.sh` runs the Panda MoveIt reference machine against the MoveIt infrastructure alone and
reports how the run and the shutdown went. `panda_infra.launch.py` is that infrastructure:
`move_group`, `ros2_control` with mock hardware, the state publisher and the three controllers,
without RViz, terminal windows, the keyboard or the machine. The machine binary is started directly so
its exit status is observed. Developer machine only, never on a pull request.

```bash
cd <workspace>
bash src/SMACC2/smacc2_performance_tools/moveit_test_tools/panda_run.sh install /tmp/panda StMoveJoints2
```

| Argument | |
|--|--|
| install prefix | holds `sm_panda_cl_moveit2z_cb_inventory` and `cl_moveit2z` |
| output directory | receives `sm.log`, `infra.log` |
| end state | the state at which the machine is stopped, any state name of the machine |
| `tsan` or `asan`, overlay prefix | optional, see below |

`MAX_WAIT` (default 600) is the number of seconds to wait for the end state, `PKG` the machine.
Needs `moveit_resources_panda_moveit_config`. Runs in ROS domain 47 unless `ROS_DOMAIN_ID` is
set. Exit status 0 when the end state was reached and the machine stopped without hanging.

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

The extra line counts the transitions taken because a client behavior reported a failure.

## Under a sanitizer

The simulator and its ROS packages stay uninstrumented; only the SMACC2 packages are rebuilt, into
their own build and install bases, as an overlay on the normal install:

```bash
cd <workspace> && source install/setup.bash
colcon build --build-base build_tsan --install-base install_tsan --packages-select smacc2 cl_moveit2z sm_panda_cl_moveit2z_cb_inventory \
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
