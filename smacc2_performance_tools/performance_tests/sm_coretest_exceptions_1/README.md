# sm_coretest_exceptions_1

Core test for user code that throws. It is not a reference state machine; it exists to run the
core's exception handling on demand and to pin what the core does when a state, a client behavior
or an `update()` throws.

## What it does

```
StA ──▶ SsPlain[ StRest ] ──▶ SsEscalate[ StInner ] ──▶ StRetry ──▶ StA      (500 ms timers)
                                  │
                                  └── EvStInitializationFailure<SsEscalate> ──▶ StSafe ──▶ StA
```

The environment variable `SM_CORETEST_THROW_SITE` names the one site that throws in a run; each
throw is announced with a `THROWING at <site>` line. With `none`, the default, nothing throws.

### Sites the core logs and continues from

All in `StA` and its behaviors. The machine keeps cycling.

| Site | Where it throws | What the core logs |
|--|--|--|
| `state_exit` | `StA::onExit` | `exception in onExit` |
| `sync_entry` | `CbSyncThrow::onEntry` | `Exception on Entry - continuing with next client behavior` |
| `sync_exit` | `CbSyncThrow::onExit` | `Exception onExit - continuing with next client behavior` |
| `async_entry` | `CbAsyncThrow::onEntry`, on the behavior's thread | `onEntry thread ended by exception`, when the state exits and the thread is joined; the behavior posts no success or failure event |
| `async_exit` | `CbAsyncThrow::onExit`, on the behavior's exit thread | `onExit thread ended by exception` |
| `client_update` | every tenth `ClThrowingUpdate::update` | `Error in updatable elemnent` |
| `behavior_update` | every tenth `CbUpdateThrow::update` | `Exception during Signal Detector update loop` |

### Failed state initializations

A state whose `runtimeConfigure` or `onEntry` throws is logged (`exception in state
initialization`), none of its client behaviors, state reactors or event generators is started or
later exited, its `onExit` does not run, and `EvStInitializationFailure<TState>` is posted. The
event is addressed to the failing state first; if no state reacts, the state machine posts it again
addressed to each container state in turn, and with no reaction anywhere it calls its
`onUnhandledStInitializationFailure`, whose default logs and leaves the machine where it is.

| Site | Where it throws | Who reacts | Result |
|--|--|--|--|
| `retry_entry` | `StRetry::onEntry`, two entries out of three | `StRetry`, by entering itself again | the third attempt gets in; the machine keeps cycling |
| `inner_entry` | `StInner::onEntry` | `SsEscalate`, its container | `StSafe`, then the cycle goes on |
| `inner_configure` | `StInner::runtimeConfigure` | `SsEscalate` | the same |
| `container_entry` | `SsEscalate::onEntry` | `SsEscalate` itself | `StInner` is not initialized either; `StSafe`, then the cycle goes on |
| `rest_entry` | `StRest::onEntry`, inside `SsPlain` | nobody | passes through `SsPlain` to the state machine's hook; the machine stays in `StRest` with nothing running |
| `state_entry` | `StA::onEntry`, top level | nobody | the hook; the machine stays in `StA` with nothing running |

The test's state machine overrides the hook to log `HOOK onUnhandledStInitializationFailure`.

## Run

```bash
SM_CORETEST_THROW_SITE=inner_entry ros2 launch sm_coretest_exceptions_1 sm_coretest_exceptions_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_exceptions_1_latest.log`.

## In the gate

`.github/scripts/sanitizer_core_tests.sh` runs all thirteen sites for 8 s each under TSan, ASan and
a plain build. It fails if a continuing site stops cycling, if a container reaction does not reach
`StSafe` once per throw, if an unhandled failure does not call the hook exactly once, if any run
ends with an uncaught exception, or on a sanitizer report with a SMACC2 frame.
