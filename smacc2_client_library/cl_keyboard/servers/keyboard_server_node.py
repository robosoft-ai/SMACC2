#!/bin/env python3

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

# *****************************************************************************************************************
# *
# * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
# *
# ******************************************************************************************************************/

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16

# Arrow keys arrive as 3-byte escape sequences (ESC [ A..D). They are published
# as the unicode arrow codepoints, which fit the UInt16 topic above the plain
# char range so downstream char-based dispatch is unaffected.
ESC = 0x1B
CTRL_C = 0x03
ARROW_CODES = {
    ord("A"): 8593,  # up    U+2191
    ord("B"): 8595,  # down  U+2193
    ord("C"): 8594,  # right U+2192
    ord("D"): 8592,  # left  U+2190
}

# How long a partial escape sequence may sit in the pending buffer before it is
# flushed as a literal ESC keypress (a human pressing the Esc key, not an arrow).
PENDING_ESC_FLUSH_SEC = 0.10


def decode_keys(buf, now, pending_since):
    """Decode a byte buffer into published key codes.

    Returns (codes, remainder, pending_since):
    - codes: list of UInt16 values to publish, in order
    - remainder: undecoded trailing bytes (partial escape sequence) to retry
      once more input arrives
    - pending_since: timestamp the remainder started waiting, or None

    A lone ESC (or unfinished CSI sequence) is held back until
    PENDING_ESC_FLUSH_SEC elapses, then flushed as a literal ESC. Complete CSI
    sequences that are not arrows (Home/End/F-keys/...) are swallowed whole so
    they never spray junk codes into the stream.
    """
    codes = []
    i = 0
    n = len(buf)
    while i < n:
        b = buf[i]
        if b != ESC:
            codes.append(b)
            i += 1
            continue

        # ESC with nothing after it (yet): flush as literal ESC only after the
        # grace period; otherwise keep waiting for the rest of the sequence.
        if i + 1 >= n:
            if pending_since is not None and (now - pending_since) >= PENDING_ESC_FLUSH_SEC:
                codes.append(ESC)
                i += 1
                pending_since = None
                continue
            return codes, buf[i:], (pending_since if pending_since is not None else now)

        if buf[i + 1] != ord("["):
            # ESC followed by a non-CSI byte: literal ESC, reprocess next byte
            codes.append(ESC)
            i += 1
            continue

        # CSI sequence: ESC [ <parameter bytes 0x30-0x3F> <final byte 0x40-0x7E>
        j = i + 2
        while j < n and 0x30 <= buf[j] <= 0x3F:
            j += 1
        if j >= n:
            # incomplete sequence: wait for more bytes (same grace period)
            if pending_since is not None and (now - pending_since) >= PENDING_ESC_FLUSH_SEC:
                codes.append(ESC)
                i += 1
                pending_since = None
                continue
            return codes, buf[i:], (pending_since if pending_since is not None else now)

        final = buf[j]
        if j == i + 2 and final in ARROW_CODES:
            codes.append(ARROW_CODES[final])
        # any other complete CSI sequence is deliberately swallowed
        i = j + 1

    return codes, b"", None


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_node")
        self.pub = self.create_publisher(UInt16, "keyboard_unicode", 10)
        # fast non-blocking poll: raw mode is held for the node lifetime and
        # every buffered byte is drained per tick, so keyboard autorepeat flows
        # at the terminal's native rate (the twist-teleop deadman depends on it)
        self.timer = self.create_timer(0.02, self.timer_update)
        self.pending = b""
        self.pending_since = None

    def timer_update(self):
        buf = self.pending
        fd = sys.stdin.fileno()
        while select.select([sys.stdin], [], [], 0)[0]:
            chunk = os.read(fd, 64)
            if not chunk:
                break
            buf += chunk

        if not buf:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        codes, self.pending, self.pending_since = decode_keys(buf, now, self.pending_since)

        for code in codes:
            if code == CTRL_C:
                # raw mode disables ISIG, so Ctrl-C arrives as a byte
                self.get_logger().info("Ctrl-C: shutting down")
                raise KeyboardInterrupt
            msg = UInt16()
            msg.data = code
            self.get_logger().info(f"key: {msg.data}")
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    settings = termios.tcgetattr(sys.stdin)
    node = KeyboardPublisher()
    try:
        tty.setraw(sys.stdin.fileno())
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
