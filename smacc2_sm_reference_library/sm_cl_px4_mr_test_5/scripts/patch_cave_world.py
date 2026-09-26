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

"""Turn a DARPA SubT world from Gazebo Fuel into one PX4 SITL can fly in.

  patch_cave_world.py <fuel world.sdf> <output.sdf>

- rewrites fuel.ignitionrobotics.org -> fuel.gazebosim.org in every <uri>
- drops the artifact props (ropes, helmets, backpacks, phones, rescue randys,
  rock falls), the inline performer-detector models (they use
  libignition-gazebo-* plugins that do not exist in Gazebo Harmonic) and the
  SubT levels plugin
- inserts the <spherical_coordinates> block PX4's NavSat sensor needs (the
  same reference as PX4's default.sdf so ref_lat/ref_lon match the other tests)

World plugins (physics, sensors, navsat, ...) are NOT added: PX4's
server.config (GZ_SIM_SERVER_CONFIG_PATH from build/px4_sitl_default/rootfs/gz_env.sh)
loads them into any world.
"""

import re
import sys
import xml.etree.ElementTree as ET

ARTIFACT_PREFIXES = (
    "rope_",
    "helmet_",
    "rescue_randy_",
    "backpack_",
    "phone_",
    "medium_rock_fall_",
    "large_rock_fall_",
)

SPHERICAL = """
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>26.478999</latitude_deg>
      <longitude_deg>56.538333</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    src, dst = sys.argv[1], sys.argv[2]

    tree = ET.parse(src)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        print("no <world> element in", src)
        return 1

    dropped = []
    for inc in list(world.findall("include")):
        name = inc.findtext("name", "")
        if name.startswith(ARTIFACT_PREFIXES):
            world.remove(inc)
            dropped.append(name)
    for model in list(world.findall("model")):
        name = model.get("name", "")
        if name.startswith("performer_detector"):
            world.remove(model)
            dropped.append(name)
    # the SubT "levels" plugin (ignition::gazebo / dummy) only matters with --levels
    # and makes gz-sim 8 log a load error
    for plugin in list(world.findall("plugin")):
        if plugin.get("filename") == "dummy":
            world.remove(plugin)
            dropped.append("levels plugin")

    rewritten = 0
    for uri in world.iter("uri"):
        if uri.text and "fuel.ignitionrobotics.org" in uri.text:
            uri.text = uri.text.replace("fuel.ignitionrobotics.org", "fuel.gazebosim.org")
            rewritten += 1

    if world.find("spherical_coordinates") is None:
        world.append(ET.fromstring(SPHERICAL))

    tree.write(dst, xml_declaration=True, encoding="utf-8")
    # ElementTree drops the newline after the declaration; keep the file readable
    text = open(dst).read()
    text = re.sub(r"\?>\s*<sdf", "?>\n<sdf", text, count=1)
    open(dst, "w").write(text + ("\n" if not text.endswith("\n") else ""))

    print(f"world: {world.get('name')}")
    print(f"uris rewritten: {rewritten}")
    print(f"dropped: {len(dropped)} ({', '.join(dropped[:6])}{', ...' if len(dropped) > 6 else ''})")
    print(f"written: {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
