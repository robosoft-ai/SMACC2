#!/usr/bin/env python3

# Copyright 2026 RobosoftAI Inc.
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

"""Classify sanitizer reports found in run logs and TSan report files.

Usage: sanitizer_reports.py <file>...
Prints every report block. Exits 1 if any block has a frame in SMACC2 code (the core, the client,
reactor and generator libraries, or the core test packages); reports whose frames are all in rclcpp,
the DDS layer or the C++ runtime are printed as external and do not fail the run.
"""
import re
import sys

SMACC_FRAME = re.compile(
    r"smacc2|sm_coretest|cl_ros2_timer|sr_event_countdown|sr_all_events_go|sr_conditional|"
    r"eg_conditional_generator|eg_random_generator"
)
START = re.compile(
    r"^(WARNING: ThreadSanitizer: .*|==\d+==ERROR: (AddressSanitizer|LeakSanitizer): .*|"
    r"(Direct|Indirect) leak of .*|.*: runtime error: .*)$"
)


def blocks(text):
    """Split a log into report blocks: a start line and the frames that follow it."""
    out, cur = [], None
    for line in text.splitlines():
        if START.match(line):
            if cur:
                out.append(cur)
            cur = [line]
        elif cur is not None:
            if line.startswith("SUMMARY:") or line.startswith("=================="):
                out.append(cur)
                cur = None
            else:
                cur.append(line)
    if cur:
        out.append(cur)
    return out


TSAN_CONTEXT = re.compile(r"^  (Location is|Thread T\d+ |Mutex M\d+ |As if synchronized)")


def access_section(block):
    """For a TSan data race, keep only the access stacks: frames under 'Location is', 'Thread T created by'
    and 'Mutex M' describe where the object or thread came from, not who raced on it. Other reports,
    including lock-order inversions (whose stacks all start with 'Mutex M'), are kept whole."""
    if not block[0].startswith("WARNING: ThreadSanitizer: data race"):
        return block
    out = []
    for line in block:
        if TSAN_CONTEXT.match(line):
            break
        out.append(line)
    return out


def main(paths):
    failing, external = [], []
    for path in paths:
        try:
            text = open(path, errors="replace").read()
        except OSError:
            continue
        for block in blocks(text):
            head = block[0].strip()
            if head.startswith("SUMMARY"):
                continue
            frames = [l for l in access_section(block) if re.match(r"\s*#\d+ ", l)]
            smacc = [l.strip() for l in frames if SMACC_FRAME.search(l)]
            entry = (
                path,
                head,
                smacc[0] if smacc else (frames[0].strip() if frames else "(no frames)"),
            )
            (failing if smacc else external).append(entry)
    for path, head, frame in external:
        print(f"external report (not failing): {head}\n    first frame: {frame}\n    in {path}")
    for path, head, frame in failing:
        print(f"FAIL: {head}\n    SMACC2 frame: {frame}\n    in {path}")
    print(f"sanitizer reports: {len(failing)} with a SMACC2 frame, {len(external)} external")
    return 1 if failing else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
