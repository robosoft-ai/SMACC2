# Package Release Summary

| Category | Packages | apt Release |
|----------|----------|:-----------:|
| Core framework | `smacc2`, `smacc2_msgs` | ✅ |
| Core libraries | `sr_*`, `eg_*` | ✅ |
| Clients | `cl_keyboard`, `cl_ros2_timer`, `cl_lifecycle_node` | ✅ |
| Reference state machines | `sm_atomic`, `sm_atomic_mode_states`, `sm_data_sharing_*`, `sm_three_some`, `sm_simple_action_client`, `sm_cl_keyboard_unit_test_1`, `sm_cl_ros2_timer_unit_test_1`, `sm_mode_state_behavior_1`, `sm_retry_logic_1` | ✅ |
| Perf tool | `sm_coretest_transition_speed_1` | ✅ |
| Nav clients + planners | `cl_nav2z`, `*_planner` packages | ❌ |
| Other clients | `cl_http`, `cl_generic_sensor`, `cl_moveit2z`, `cl_mission_tracker`, `cl_foundation_pose`, `cl_isaac_apriltag`, `cl_gcalcli`, `cl_px4_mr`, `cl_modbus_tcp_relay` | ❌ |
| Other reference SMs | `sm_atomic_http`, `sm_atomic_lifecycle`, `sm_branching`, `sm_multi_stage_1`, `sm_multithread_test_1`, `sm_advanced_recovery_1`, `sm_pack_ml` | ❌ |
| Test/demo state machines | `sm_cl_gcalcli_test_1`, `sm_cl_px4_mr_*`, `sm_modbus_*`, `sm_nav2_gazebo_*`, `sm_panda_*` | ❌ |
| Perf tools (unused) | `sm_atomic_performance_trace_1`, `sm_atomic_subscribers_performance_test` | ❌ |

---

# Package Release List

This table is the authoritative list of which packages are included in each apt release.
Update it here before running bloom. Packages marked ❌ are removed from the rosdistro
PR after bloom runs (see Phase 5: Post-Bloom Package Filtering).

| Package | Category | apt Release | Reason |
|---------|----------|:-----------:|--------|
| `smacc2` | Core | ✅ | Core framework |
| `smacc2_msgs` | Core | ✅ | Core messages |
| `sr_all_events_go` | State Reactor | ✅ | Core library |
| `sr_conditional` | State Reactor | ✅ | Core library |
| `sr_event_countdown` | State Reactor | ✅ | Core library |
| `eg_conditional_generator` | Event Generator | ✅ | Core library |
| `eg_random_generator` | Event Generator | ✅ | Core library |
| `cl_keyboard` | Client | ✅ | General purpose |
| `cl_ros2_timer` | Client | ✅ | General purpose |
| `cl_lifecycle_node` | Client | ✅ | Core ROS2 lifecycle management |
| `cl_http` | Client | ❌ | Not in current release scope |
| `cl_nav2z` | Client | ❌ | Not in current release scope |
| `backward_global_planner` | Nav Planner | ❌ | Not in current release scope |
| `backward_local_planner` | Nav Planner | ❌ | Not in current release scope |
| `forward_global_planner` | Nav Planner | ❌ | Not in current release scope |
| `forward_local_planner` | Nav Planner | ❌ | Not in current release scope |
| `nav2z_planners_common` | Nav Planner | ❌ | Not in current release scope |
| `pure_spinning_local_planner` | Nav Planner | ❌ | Not in current release scope |
| `undo_path_global_planner` | Nav Planner | ❌ | Not in current release scope |
| `cl_generic_sensor` | Client | ❌ | Not in current release scope |
| `cl_moveit2z` | Client | ❌ | Not in current release scope |
| `cl_mission_tracker` | Client | ❌ | Too specialized |
| `cl_foundation_pose` | Client | ❌ | Isaac-specific |
| `cl_isaac_apriltag` | Client | ❌ | Isaac-specific |
| `cl_gcalcli` | Client | ❌ | Google Calendar — too specialized |
| `cl_px4_mr` | Client | ❌ | PX4 drone hardware |
| `cl_modbus_tcp_relay` | Client | ❌ | Modbus hardware |
| `sm_atomic` | Reference SM | ✅ | Canonical minimal example |
| `sm_atomic_mode_states` | Reference SM | ✅ | Mode states example |
| `sm_data_sharing_1` | Reference SM | ✅ | Data sharing pattern |
| `sm_data_sharing_2` | Reference SM | ✅ | Data sharing pattern |
| `sm_three_some` | Reference SM | ✅ | Multi-orthogonal example |
| `sm_simple_action_client` | Reference SM | ✅ | Action client example |
| `sm_cl_keyboard_unit_test_1` | Reference SM | ✅ | Keyboard example |
| `sm_cl_ros2_timer_unit_test_1` | Reference SM | ✅ | Timer example |
| `sm_mode_state_behavior_1` | Reference SM | ✅ | Mode state behavior pattern |
| `sm_retry_logic_1` | Reference SM | ✅ | Retry pattern |
| `sm_coretest_transition_speed_1` | Perf Tool | ✅ | Transition speed benchmarking |
| `sm_atomic_http` | Reference SM | ❌ | Not in current release scope |
| `sm_atomic_lifecycle` | Reference SM | ❌ | Not in current release scope |
| `sm_branching` | Reference SM | ❌ | Not in current release scope |
| `sm_multi_stage_1` | Reference SM | ❌ | Not in current release scope |
| `sm_multithread_test_1` | Reference SM | ❌ | Not in current release scope |
| `sm_advanced_recovery_1` | Reference SM | ❌ | Not in current release scope |
| `sm_pack_ml` | Reference SM | ❌ | Needs rehaul before releasing |
| `sm_cl_gcalcli_test_1` | Test SM | ❌ | Requires Google Calendar |
| `sm_cl_px4_mr_test_1` | Test SM | ❌ | Requires PX4 drone |
| `sm_cl_px4_mr_test_2` | Test SM | ❌ | Requires PX4 drone |
| `sm_modbus_tcp_relay_test_1` | Test SM | ❌ | Requires Modbus hardware |
| `sm_nav2_gazebo_test_1` | Test SM | ❌ | Requires Gazebo + Nav2 hardware |
| `sm_panda_cl_moveit2z_cb_inventory` | Test SM | ❌ | Requires Panda arm |
| `sm_panda_cl_moveit2z_cb_inventory_isaacsim` | Test SM | ❌ | Requires Isaac Sim |
| `sm_atomic_performance_trace_1` | Perf Tool | ❌ | Internal testing only |
| `sm_atomic_subscribers_performance_test` | Perf Tool | ❌ | Internal testing only |

> **New packages:** When bloom adds a new package to the rosdistro PR, add a row here
> before merging. Default to ❌ until reviewed.

---

# SMACC2 Release Process

**Division of Labor:**
- **Claude**: Analyzes code, bumps versions, generates CHANGELOG, runs `filter_rosdistro.py`, drafts PR descriptions and release notes
- **Human**: Executes git commands on host, runs bloom-release, creates GitHub PR, publishes release

## Prerequisites (Host System)

```bash
# Install bloom (apt version is often outdated)
pip3 install --user bloom
export PATH="$HOME/.local/bin:$PATH"

# Verify
which bloom-release
gh auth status
```

Required repo access: `robosoft-ai/SMACC2`, `robosoft-ai/SMACC2-release`, fork of `ros/rosdistro`.

## Release Phases

### Phase 1: Pre-Release Verification (Claude)

Claude will:
1. Verify the fix/feature exists in source
2. Review commits since last release
3. Determine version number (see Version Numbering below)
4. Categorize changes (fixes, features, breaking changes)

### Phase 2: Version & Documentation Updates (Claude)

Claude will:
1. Update all `package.xml` files to new version
2. Generate CHANGELOG.rst entry
3. Prepare commit message

### Phase 3: Commit, Tag, and Push (Human)

```bash
git add -A
git commit -m "Prepare release X.Y.Z"
git tag -a X.Y.Z -m "Release X.Y.Z - [summary]"
git push origin your-branch-name
git push origin X.Y.Z
# Open PR to jazzy branch, merge it
```

**After merge:** Verify the tag is on the jazzy branch. If it landed on a feature branch, re-create it:
```bash
git checkout jazzy && git pull origin jazzy
git tag -d X.Y.Z && git push origin :refs/tags/X.Y.Z
git tag -a X.Y.Z -m "Release X.Y.Z - [summary]"
git push origin X.Y.Z
```

### Phase 4: Bloom Release (Human)

```bash
# Fix rosdep if needed
sudo mv /etc/ros/rosdep/sources.list.d/10-debian.list \
        /etc/ros/rosdep/sources.list.d/10-debian.list.disabled
rosdep update

# If bloom picks up the wrong version, fix tracks.yaml first:
cd /tmp && git clone https://github.com/robosoft-ai/SMACC2-release.git && cd SMACC2-release
sed -i 's/last_version: OLD_VERSION/last_version: X.Y.Z/g' tracks.yaml
git add tracks.yaml && git commit -m "Update tracks.yaml to X.Y.Z"

# Run bloom
bloom-release smacc2 --rosdistro jazzy --track jazzy --unsafe
# Prompt: missing optional dependencies → 'y'
# Prompt: create automatic PR → 'y' (will likely fail with 403; proceed to Phase 5)
```

### Phase 5: rosdistro PR (Human + Claude)

```bash
cd /tmp
git clone https://github.com/brettpac/rosdistro.git && cd rosdistro
git remote add upstream https://github.com/ros/rosdistro.git
git fetch upstream
git checkout -b smacc2-X.Y.Z-release upstream/master

# Update version
sed -i 's/version: OLD_VERSION-REV/version: X.Y.Z-REV/g' jazzy/distribution.yaml
```

**Claude runs the package filter:**
```bash
python3 /path/to/SMACC2/.github/filter_rosdistro.py jazzy/distribution.yaml
# Preview only: add --dry-run
```

The approved list is maintained in `.github/filter_rosdistro.py` (`APPROVED_PACKAGES`) and
mirrored in the Package Release List table above.

```bash
git add jazzy/distribution.yaml
git commit -m "smacc2: X.Y.Z-REV in 'jazzy/distribution.yaml' [bloom]"
git push origin smacc2-X.Y.Z-release
```

**Claude drafts the PR description; human creates PR on GitHub:**
- Base: `ros/rosdistro:master`
- Title: `smacc2: X.Y.Z-REV in 'jazzy/distribution.yaml' [bloom]`

### Phase 6: GitHub Release & Communication (Claude + Human)

Claude drafts; human publishes:
1. GitHub release notes at `github.com/robosoft-ai/SMACC2/releases/new` (select tag X.Y.Z)
2. Comment on any issues fixed by this release

### Phase 7: Monitor

| Day | Event |
|-----|-------|
| 0 | rosdistro PR submitted |
| 1-2 | ROS maintainers merge PR |
| 2-3 | Buildfarm compiles packages |
| 3-4 | Packages sync to apt.ros.org |

```bash
apt-cache policy ros-jazzy-smacc2  # check version when available
```

- Buildfarm: https://build.ros2.org/
- rosdistro PRs: https://github.com/ros/rosdistro/pulls

## Common Issues

### Bloom uses wrong version

```bash
cd /tmp/SMACC2-release
sed -i 's/last_version: OLD_VERSION/last_version: X.Y.Z/g' tracks.yaml
git add tracks.yaml && git commit -m "Update tracks.yaml to X.Y.Z"
bloom-release smacc2 --rosdistro jazzy --track jazzy --unsafe
```

### rosdep update fails

```bash
sudo mv /etc/ros/rosdep/sources.list.d/10-debian.list \
        /etc/ros/rosdep/sources.list.d/10-debian.list.disabled
rosdep update
```

### Missing dependency prompt during bloom

- Optional/demo package dependency → answer `y`
- Critical dependency → fix before proceeding

## Version Numbering

- **PATCH** (X.Y.Z+1): Bug fixes, no API changes
- **MINOR** (X.Y+1.0): New features, backward compatible
- **MAJOR** (X+1.0.0): Breaking changes

**Debian revision** (`-REV`): Starts at `-1`, increment if re-releasing the same upstream version.

## Release Checklist

- [ ] Claude verifies fix/feature in code
- [ ] Claude determines version number
- [ ] Claude updates all `package.xml` files
- [ ] Claude generates CHANGELOG entry
- [ ] Human commits, tags, pushes, merges PR to jazzy
- [ ] Human runs bloom-release
- [ ] Claude runs `filter_rosdistro.py` on distribution.yaml
- [ ] Human commits filtered distribution.yaml and creates rosdistro PR
- [ ] Claude drafts GitHub release notes
- [ ] Human publishes GitHub release and posts issue updates
- [ ] Human monitors rosdistro PR merge (1-2 days)
- [ ] Human verifies apt package availability

## Reference Links

- Release tracks: https://github.com/robosoft-ai/SMACC2-release/blob/master/tracks.yaml
- rosdistro entry: https://github.com/ros/rosdistro/blob/master/jazzy/distribution.yaml
- Buildfarm: https://build.ros2.org/
- Bloom docs: http://wiki.ros.org/bloom
- ROS2 Release Process: https://docs.ros.org/en/rolling/How-To-Guides/Releasing/First-Time-Release.html
