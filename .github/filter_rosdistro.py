#!/usr/bin/env python3
# Copyright 2025 Robosoft Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Filter smacc2 packages in jazzy/distribution.yaml to the approved allowlist.

Run this after bloom creates the rosdistro PR branch, before committing:
  python3 filter_rosdistro.py /path/to/rosdistro/jazzy/distribution.yaml

Use --dry-run to preview changes without modifying the file.

The approved list is the source of truth for which packages get apt releases.
To change the list, update RELEASING.md and this file together.
"""

import argparse
import re
import sys

# Packages approved for apt release. See RELEASING.md for rationale.
APPROVED_PACKAGES = {
    # Core framework
    "smacc2",
    "smacc2_msgs",
    # State reactors
    "sr_all_events_go",
    "sr_conditional",
    "sr_event_countdown",
    # Event generators
    "eg_conditional_generator",
    "eg_random_generator",
    # Clients
    "cl_keyboard",
    "cl_ros2_timer",
    "cl_lifecycle_node",
    # Reference state machines
    "sm_atomic",
    "sm_atomic_mode_states",
    "sm_data_sharing_1",
    "sm_data_sharing_2",
    "sm_three_some",
    "sm_simple_action_client",
    "sm_cl_keyboard_unit_test_1",
    "sm_cl_ros2_timer_unit_test_1",
    "sm_mode_state_behavior_1",
    "sm_retry_logic_1",
    # Performance
    "sm_coretest_transition_speed_1",
}


def filter_distribution(path, dry_run=False):
    with open(path) as f:
        lines = f.readlines()

    in_smacc2 = False
    in_release = False
    in_packages = False
    removed = []
    found = set()
    output = []

    for line in lines:
        # Enter/exit the smacc2 top-level entry (2-space indent)
        if re.match(r"^  smacc2:\s*$", line):
            in_smacc2 = True
            in_release = False
            in_packages = False
        elif in_smacc2 and re.match(r"^  \w", line):
            in_smacc2 = False
            in_release = False
            in_packages = False

        if in_smacc2:
            if re.match(r"^    release:\s*$", line):
                in_release = True
            elif in_release and re.match(r"^      packages:\s*$", line):
                in_packages = True
            elif in_packages:
                m = re.match(r"^      - (\S+)\s*$", line)
                if m:
                    pkg = m.group(1)
                    found.add(pkg)
                    if pkg not in APPROVED_PACKAGES:
                        removed.append(pkg)
                        continue  # drop line regardless of dry_run; report below
                else:
                    in_packages = False

        output.append(line)

    missing = sorted(APPROVED_PACKAGES - found)

    if not dry_run:
        with open(path, "w") as f:
            f.writelines(output)

    prefix = "[DRY RUN] " if dry_run else ""

    print(f"{prefix}Removed {len(removed)} packages:")
    for pkg in sorted(removed):
        print(f"  - {pkg}")

    if missing:
        print(f"\nWARNING: {len(missing)} approved packages not found in distribution.yaml:")
        for pkg in missing:
            print(f"  ? {pkg}")
        print("  These may be new packages not yet in rosdistro, or renamed.")
        print("  Verify before merging the PR.")

    retained = len(found & APPROVED_PACKAGES)
    print(f"\nRetained {retained}/{len(APPROVED_PACKAGES)} approved packages.")
    if not dry_run and removed:
        print(f"File updated: {path}")


def main():
    parser = argparse.ArgumentParser(
        description="Filter smacc2 packages in jazzy/distribution.yaml to the approved allowlist."
    )
    parser.add_argument("distribution_yaml", help="Path to jazzy/distribution.yaml")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview removals without modifying the file",
    )
    args = parser.parse_args()
    filter_distribution(args.distribution_yaml, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
