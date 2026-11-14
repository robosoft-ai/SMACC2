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

"""Send a scripted sequence of keypresses to /keyboard_unicode from ONE publisher.

Usage: send_keys.py <step> [<step> ...]     e.g.  send_keys.py n*5 f n*5 pause:1
  <char>          one keypress, then one second
  <char>*<count>  that keypress <count> times, one per second
  pause:<secs>    wait
  period:<secs>   the time between keypresses from here on (default 1)

One publisher for the whole sequence, with a settling second after it is matched: the first message
of a publisher that has only just been matched can be lost, and a new `ros2 topic pub` per group of
keys starts a new publisher each time.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16


def main():
    rclpy.init()
    node = Node("regression_key_sender")
    pub = node.create_publisher(UInt16, "/keyboard_unicode", 10)
    deadline = time.time() + 30.0
    while pub.get_subscription_count() < 1 and time.time() < deadline:
        time.sleep(0.05)
    if pub.get_subscription_count() < 1:
        print("send_keys: no subscriber on /keyboard_unicode after 30 s", file=sys.stderr)
        sys.exit(1)
    time.sleep(1.0)
    sent = 0
    period = 1.0
    for step in sys.argv[1:]:
        if step.startswith("pause:"):
            time.sleep(float(step[6:]))
            continue
        if step.startswith("period:"):
            period = float(step[7:])
            continue
        char, _, count = step.partition("*")
        for _ in range(int(count or 1)):
            pub.publish(UInt16(data=ord(char)))
            sent += 1
            time.sleep(period)
    time.sleep(0.5)
    print(f"send_keys: {sent} keypresses sent")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
