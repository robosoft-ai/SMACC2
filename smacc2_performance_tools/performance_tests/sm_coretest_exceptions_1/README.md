# sm_coretest_exceptions_1

Core test for user code that throws. It is not a reference state machine; it exists to run the
core's exception handlers on demand and to pin what the core does when a state, a client behavior
or an `update()` throws.

## What it does

```
StA  ──EvTimer<CbTimerCountdownOnce> (500 ms)──▶  StRest  ──EvTimer<CbTimerCountdownLoop> (500 ms)──▶  StA
```

`StA` holds every throw site. The environment variable `SM_CORETEST_THROW_SITE` names the one site
that throws in a run; each throw is announced with a `THROWING at <site>` line. `StRest` throws
nothing, so reaching it again and again shows the machine survived the throw.

| Site | Where it throws | What the core does |
|--|--|--|
| `none` (default) | nowhere | |
| `state_exit` | `StA::onExit` | logs `exception in onExit`, the exit completes |
| `sync_entry` | `CbSyncThrow::onEntry` | logs `Exception on Entry - continuing with next client behavior` |
| `sync_exit` | `CbSyncThrow::onExit` | logs `Exception onExit - continuing with next client behavior` |
| `async_entry` | `CbAsyncThrow::onEntry`, on the behavior's thread | logs `onEntry thread ended by exception` when the state exits and the thread is joined; the behavior posts no success or failure event |
| `async_exit` | `CbAsyncThrow::onExit`, on the behavior's exit thread | logs `onExit thread ended by exception` |
| `client_update` | every tenth `ClThrowingUpdate::update` | logs `Error in updatable elemnent`, the polling loop continues |
| `behavior_update` | every tenth `CbUpdateThrow::update` | logs `Exception during Signal Detector update loop`, the polling loop continues |
| `state_entry` | `StA::onEntry` | **the process ends**: the exception is not caught and `std::terminate` is called |

## Run

```bash
SM_CORETEST_THROW_SITE=sync_entry ros2 launch sm_coretest_exceptions_1 sm_coretest_exceptions_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_exceptions_1_latest.log`.

## In the gate

`.github/scripts/sanitizer_core_tests.sh` runs the seven handled sites for 6 s each under TSan, ASan
and a plain build, and fails if a run takes fewer than three throws, stops alternating, ends with an
uncaught exception or produces a sanitizer report with a SMACC2 frame. `state_entry` is left out
because the process ends there today.
