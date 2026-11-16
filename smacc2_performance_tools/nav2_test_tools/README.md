# Nav2 test tools

`nav2_run.sh` runs one of the Nav2 reference machines against a headless TurtleBot3 Gazebo
simulation and reports how the run and the shutdown went. It exists because the machines' own launch
files open terminal windows and RViz and hide the machine's exit status; here the machine binary is
started directly. Developer machine only, never on a pull request.

```bash
cd <workspace>
bash src/SMACC2/smacc2_performance_tools/nav2_test_tools/nav2_run.sh install /tmp/nav2_1 1
```

| Argument | |
|--|--|
| install prefix | holds `sm_nav2_gazebo_test_1` to `_4` and `cl_nav2z` |
| output directory | receives `sm.log`, `bringup.log`, `cpu.txt` |
| `1`, `2`, `3` or `4` | which machine; 2 and 3 use their own world, map and Nav2 parameters, 4 uses those of 2, and 3 is sent the `n` key while it waits in its teleop state |
| `tsan` or `asan`, overlay prefix | optional, see below |

Needs `nav2_bringup` with the TurtleBot3 simulation and Gazebo. Machine 1 takes about a minute; the
script gives up after 20 minutes, or after 120 s without a new state. Exit status 0 when
`StFinalState` was reached and the machine stopped without hanging.

## Reading the report

| Line | Meaning |
|--|--|
| `verdict` | whether the end state was reached, and after how long; `died` means the machine exited by itself, `timeout` or `stall` that it stopped progressing, `failed` that machine 4 ended in `StMissionFailed` |
| `SM exit` | the machine's exit status after SIGINT and how long shutdown took; `SM HANG` means it was still alive 120 s later |
| `leftover after` | simulator or node processes still running when the script ended; should be 0 |
| `== states` | every state entered, in order |
| `== shutdown` | the shutdown markers the core logged, in order |
| `== ERROR ...` | counts of error and fatal log lines, discarded events, DDS teardown errors and crashes, then the most frequent error lines |

All logs are in the output directory: `sm.log` for the machine, one log per simulator process.

Error lines from the pose component about the `map` frame at the start of a run are normal: the
machine starts before localization has published the transform.

## Under a sanitizer

The simulator and its ROS packages stay uninstrumented; only the SMACC2 packages are rebuilt, into
their own build and install bases, as an overlay on the normal install:

```bash
cd <workspace> && source install/setup.bash
colcon build --build-base build_tsan --install-base install_tsan --packages-select smacc2 cl_nav2z sm_nav2_gazebo_test_1 sm_nav2_gazebo_test_2 sm_nav2_gazebo_test_3 sm_nav2_gazebo_test_4 \
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
