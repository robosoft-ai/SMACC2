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

"""Offline helper: list the tiles of a SubT world and suggest a route.

  cave_route_from_sdf.py <world.sdf> [world.dot] [--spawn x,y,z] [--depth N]

Prints every <include> (name, model, pose). With the Fuel .dot file (same
directory as the world) it walks the tile graph from the base station and
prints the chain of tile centres out to the first tile whose floor height
differs from the start, as gz ENU coordinates and as PX4 NED coordinates
relative to the spawn point (north = gz y, east = gz x). Paste the result
into config/mission_constants.hpp and refine it against the RViz cloud.
"""

import re
import sys
import xml.etree.ElementTree as ET
from collections import deque


def parse_world(path):
    tiles = {}
    world = ET.parse(path).getroot().find("world")
    for inc in world.findall("include"):
        name = inc.findtext("name", "")
        pose = [float(v) for v in inc.findtext("pose", "0 0 0 0 0 0").split()]
        model = inc.findtext("uri", "").rsplit("/", 1)[-1]
        tiles[name] = (model, pose)
    return tiles


def parse_dot(path):
    labels, edges = {}, {}
    for line in open(path):
        m = re.match(r'\s*(\d+)\s*\[label="\d+::(.*?)::(.*?)"\]', line)
        if m:
            # tiles are "N::Model::tile_k"; the base station is "0::base_station::BaseStation"
            a, b = m.group(2), m.group(3)
            labels[m.group(1)] = b if b.startswith("tile_") else a
            continue
        m = re.match(r"\s*(\d+)\s*--\s*(\d+)", line)
        if m:
            a, b = m.group(1), m.group(2)
            edges.setdefault(a, []).append(b)
            edges.setdefault(b, []).append(a)
    return labels, edges


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    spawn = (2.0, 0.0, 0.25)
    depth = 8
    for a in sys.argv[1:]:
        if a.startswith("--spawn="):
            spawn = tuple(float(v) for v in a.split("=", 1)[1].split(","))
        if a.startswith("--depth="):
            depth = int(a.split("=", 1)[1])
    if not args:
        print(__doc__)
        return 2

    tiles = parse_world(args[0])
    print(f"{'name':<22}{'model':<52}pose (gz x y z yaw)")
    for name, (model, pose) in tiles.items():
        print(f"{name:<22}{model:<52}{pose[0]:8.1f} {pose[1]:8.1f} {pose[2]:7.1f} {pose[5]:6.2f}")

    if len(args) < 2:
        return 0

    labels, edges = parse_dot(args[1])
    start = next((k for k, v in labels.items() if v == "base_station"), None)
    if start is None:
        print("no base_station vertex in the .dot file")
        return 1

    # BFS: one chain, following the first unvisited neighbour each step
    chain, seen, cur = [], {start}, start
    for _ in range(depth):
        nxt = [n for n in edges.get(cur, []) if n not in seen]
        if not nxt:
            break
        cur = nxt[0]
        seen.add(cur)
        chain.append(labels[cur])

    print("\ntile chain from the base station:")
    z0 = None
    route = []
    for name in chain:
        model, pose = tiles.get(name, ("?", [0] * 6))
        if z0 is None:
            z0 = pose[2]
        flag = (
            "" if abs(pose[2] - z0) < 0.5 else "   <- floor height changes; stop before this tile"
        )
        print(f"  {name:<10}{model:<52}{pose[0]:8.1f} {pose[1]:8.1f} {pose[2]:7.1f}{flag}")
        if flag:
            break
        route.append(pose[:3])

    print("\nsuggested waypoints (gz ENU x, y | NED north, east relative to spawn):")
    for x, y, z in route:
        print(f"  {{{x:7.1f}f, {y:7.1f}f}}   NED ({y - spawn[1]:7.1f}, {x - spawn[0]:7.1f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
