# sm_coretest_features_1

Core test for the features no other core test reaches: an event generator, `CbSequence`, a behavior
that posts its failure event, and an updatable with an update period. It is not a reference state
machine; it exists so this code runs under the sanitizers.

## What it does

Six states follow each other and start again. Each sends an outcome it must not see to `StTrap`,
which is terminal and logs `TRAP ENTERED`.

| State | What runs | What must happen |
|--|--|--|
| `StGenerator` | an `EgConditionalGenerator` in update mode | it posts `EvFalse` on six updates, then `EvTrue`, the only event the state reacts to |
| `StSequence` | a `CbSequence` of three 50 ms sleeps | every step runs; the sequence posts `EvCbSuccess` |
| `StSequenceAbort` | a `CbSequence` of a sleep, `CbFail`, a sleep | `CbFail` posts its failure event; the sequence posts `EvCbFailure` and the third step never runs |
| `StPeriodic` | `CbPeriodicTick`, an updatable with a 200 ms update period, for 1.1 s | about five ticks; the 20 Hz polling loop would give about twenty-two without the period |
| `StSequenceCut` | a `CbSequence` of ten sleeps against a 330 ms timer | the state exits with the sequence part way through; no further step starts |
| `StSequenceCutOnStep` | the same against a 200 ms timer | the state exits just as the sequence starts its second step |

The last two are the reason for the test's shape. A `CbSequence` starts each step from its own
thread, and starting a step creates a behavior and signal connections, which take the state
machine's mutex. An exiting state holds that mutex while it joins the sequence's thread. Before the
fix that came with this test, `StSequenceCut` hung the machine on its first exit, for good and
deaf to SIGINT: the sequence started a new step after it had been asked to finish. With that
closed, `StSequenceCutOnStep` still hung it, because the exit began while a step was being started.

## Run

```bash
ros2 launch sm_coretest_features_1 sm_coretest_features_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_features_1_latest.log`.

## In the gate

`.github/scripts/sanitizer_core_tests.sh` runs the machine for 30 s under TSan, ASan and a plain
build, then interrupts it at five random moments. It fails if the machine completes fewer than three
cycles, if `StTrap` is entered, if a sequence does not complete or abort once per cycle, if the tick
count is not between three and seven per `StPeriodic`, if a run ends with an uncaught exception or a
hang, or on a sanitizer report with a SMACC2 frame.
