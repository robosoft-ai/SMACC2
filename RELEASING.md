# Package Release Summary

Rosdistro reviewers will only reliably approve packages with `smacc2` in the name.
All other packages are excluded from the apt release regardless of merit.

| Category | Packages | apt Release |
|----------|----------|:-----------:|
| Core framework | `smacc2`, `smacc2_msgs` | ✅ |
| Core libraries | `sr_*`, `eg_*` | ❌ |
| Clients | `cl_keyboard`, `cl_ros2_timer`, `cl_lifecycle_node` | ❌ |
| Reference state machines | `sm_atomic`, `sm_atomic_mode_states`, etc. | ❌ |
| Everything else | all remaining packages | ❌ |

---

# Package Release List

This table is the authoritative list of which packages are included in each apt release.
Update it here before running bloom. Packages marked ❌ are removed from the rosdistro
PR after bloom runs (see Phase 5: Post-Bloom Package Filtering).

| Package | Category | apt Release | Reason |
|---------|----------|:-----------:|--------|
| `smacc2` | Core | ✅ | Core framework |
| `smacc2_msgs` | Core | ✅ | Core messages |
| `sr_all_events_go` | State Reactor | ❌ | Name lacks `smacc2` prefix |
| `sr_conditional` | State Reactor | ❌ | Name lacks `smacc2` prefix |
| `sr_event_countdown` | State Reactor | ❌ | Name lacks `smacc2` prefix |
| `eg_conditional_generator` | Event Generator | ❌ | Name lacks `smacc2` prefix |
| `eg_random_generator` | Event Generator | ❌ | Name lacks `smacc2` prefix |
| `cl_keyboard` | Client | ❌ | Name lacks `smacc2` prefix |
| `cl_ros2_timer` | Client | ❌ | Name lacks `smacc2` prefix |
| `cl_lifecycle_node` | Client | ❌ | Name lacks `smacc2` prefix |
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
| `sm_atomic` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_atomic_mode_states` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_data_sharing_1` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_data_sharing_2` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_three_some` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_simple_action_client` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_cl_keyboard_unit_test_1` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_cl_ros2_timer_unit_test_1` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_mode_state_behavior_1` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_retry_logic_1` | Reference SM | ❌ | Name lacks `smacc2` prefix |
| `sm_coretest_transition_speed_1` | Perf Tool | ❌ | Name lacks `smacc2` prefix |
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
- Buildfarm root: https://build.ros2.org/
- Bloom docs: http://wiki.ros.org/bloom
- ROS2 Release Process: https://docs.ros.org/en/rolling/How-To-Guides/Releasing/First-Time-Release.html

## Buildfarm Monitoring Pages

Check these after each release to confirm all targets build successfully.

### smacc2

| Build | Link |
|-------|------|
| Jazzy · Noble · amd64 · binary | https://build.ros2.org/view/Jbin_uN64/job/Jbin_uN64__smacc2__ubuntu_noble_amd64__binary/ |
| Jazzy · Noble · arm64 · binary | https://build.ros2.org/view/Jbin_unv8_uNv8/job/Jbin_unv8_uNv8__smacc2__ubuntu_noble_arm64__binary/ |
| Jazzy · Noble · source | https://build.ros2.org/view/Jsrc_uN/job/Jsrc_uN__smacc2__ubuntu_noble__source/ |
| Jazzy · Noble · dev | https://build.ros2.org/view/Jdev/job/Jdev__smacc2__ubuntu_noble_amd64/ |
| Humble · Jammy · amd64 · binary | https://build.ros2.org/view/Hbin_uJ64/job/Hbin_uJ64__smacc2__ubuntu_jammy_amd64__binary/ |
| Humble · Jammy · arm64 · binary | https://build.ros2.org/view/Hbin_ujv8_uJv8/job/Hbin_ujv8_uJv8__smacc2__ubuntu_jammy_arm64__binary/ |

### smacc2_msgs

| Build | Link |
|-------|------|
| Jazzy · Noble · amd64 · binary | https://build.ros2.org/view/Jbin_uN64/job/Jbin_uN64__smacc2_msgs__ubuntu_noble_amd64__binary/ |
| Jazzy · Noble · arm64 · binary | https://build.ros2.org/view/Jbin_unv8_uNv8/job/Jbin_unv8_uNv8__smacc2_msgs__ubuntu_noble_arm64__binary/ |
| Jazzy · Noble · source | https://build.ros2.org/view/Jsrc_uN/job/Jsrc_uN__smacc2_msgs__ubuntu_noble__source/ |
| Humble · Jammy · amd64 · binary | https://build.ros2.org/view/Hbin_uJ64/job/Hbin_uJ64__smacc2_msgs__ubuntu_jammy_amd64__binary/ |
| Humble · Jammy · arm64 · binary | https://build.ros2.org/view/Hbin_ujv8_uJv8/job/Hbin_ujv8_uJv8__smacc2_msgs__ubuntu_jammy_arm64__binary/ |
