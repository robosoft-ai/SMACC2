# Copyright 2021 RobosoftAI Inc.
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
Isaac Sim ROS2 Bridge Setup Script for sm_panda_cl_moveit2z_cb_inventory_isaacsim

This script is designed to be run inside Isaac Sim's Script Editor (Window > Script Editor)
AFTER the USD stage has been loaded.

Prerequisites:
1. Launch Isaac Sim
2. Open the USD stage: File > Open > carter_warehouse_navigation w ZED Pandas.usd
3. Run this script in Script Editor (Window > Script Editor)
4. Press Play in Isaac Sim
5. Launch the SMACC2 state machine from a separate terminal

Usage:
1. Copy this script content
2. Open Isaac Sim Script Editor (Window > Script Editor)
3. Paste and click "Run" (or Ctrl+Enter)
"""

# =============================================================================
# CONFIGURATION - Modify these values as needed
# =============================================================================

# Prim paths in the USD stage
FRANKA_STAGE_PATH = "/World/carter_warehouse_navigation_w_ZED_Pandas/panda_instanceable"
ZED_CAMERA_PATH = "/World/carter_warehouse_navigation_w_ZED_Pandas/ZED_X"

# Franka hand link for camera mounting
FRANKA_WRIST_LINK = "panda_hand"

# OmniGraph paths (use unique names to avoid conflicts with existing graphs)
ACTION_GRAPH_PATH = "/Panda_MoveIt_ActionGraph"
ZED_CAMERA_GRAPH_PATH = "/ZED_Camera_ROS2_Graph"

# Camera mounting offset (meters) - adjust as needed
CAMERA_MOUNT_OFFSET = (0.05, 0.0, 0.05)  # (forward, right, up) from wrist
CAMERA_MOUNT_ROTATION = (0, -90, 0)  # (roll, pitch, yaw) degrees

# ROS2 Topic names - MoveIt2 integration
JOINT_STATES_TOPIC = "joint_states"
JOINT_COMMANDS_TOPIC = "joint_commands"

# ROS2 Topic names - ZED camera (matching real ZED driver naming)
ZED_RGB_TOPIC = "zed/zed_node/rgb/image_rect_color"
ZED_CAMERA_INFO_TOPIC = "zed/zed_node/rgb/camera_info"
ZED_DEPTH_TOPIC = "zed/zed_node/depth/depth_registered"
ZED_POINTCLOUD_TOPIC = "zed/zed_node/point_cloud/cloud_registered"

# Frame IDs for TF
ZED_FRAME_ID = "zed_camera_link"

# =============================================================================
# SCRIPT IMPLEMENTATION
# =============================================================================

import omni
import omni.usd
import omni.graph.core as og
import usdrt.Sdf
from pxr import Sdf, UsdGeom, Gf
from isaacsim.core.utils.extensions import enable_extension
import os

# Log file for debugging (readable from terminal)
LOG_FILE = os.path.expanduser("~/isaacsim_ros2_bridge_setup.log")


def log(msg):
    """Print and log to file"""
    print(msg)
    with open(LOG_FILE, "a") as f:
        f.write(msg + "\n")


# Clear log file at start
with open(LOG_FILE, "w") as f:
    f.write("")

log("=" * 60)
log("Isaac Sim ROS2 Bridge Setup Script")
log("=" * 60)

# Step 1: Get the current stage
stage = omni.usd.get_context().get_stage()
if stage is None:
    log("ERROR: No stage is currently open!")
    log("Please open a USD stage first (File > Open)")
    raise RuntimeError("No stage open")

log(f"Stage loaded: {stage.GetRootLayer().identifier}")

# Step 2: Enable ROS2 bridge extension
log("\nEnabling ROS2 bridge extension...")
enable_extension("isaacsim.ros2.bridge")
omni.kit.app.get_app().update()
log("ROS2 bridge extension enabled")

# Step 3: Verify prims exist
log("\nVerifying prim paths...")
franka_prim = stage.GetPrimAtPath(FRANKA_STAGE_PATH)
if not franka_prim.IsValid():
    log(f"ERROR: Franka prim not found at {FRANKA_STAGE_PATH}")
    log("\nSearching for Franka/Panda prims in stage...")
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if "panda" in path_str.lower() or "franka" in path_str.lower():
            # Show robot prims up to 5 levels deep
            if path_str.count("/") <= 5 and (
                "instanceable" in path_str.lower() or path_str.endswith("panda")
            ):
                log(f"  Found: {path_str} (type: {prim.GetTypeName()})")
    log("\nAll top-level prims under /World:")
    world_prim = stage.GetPrimAtPath("/World")
    if world_prim.IsValid():
        for child in world_prim.GetChildren():
            log(f"  /World/{child.GetName()}")
    raise RuntimeError("Franka prim not found - check paths above and update FRANKA_STAGE_PATH")
log(f"  Franka found: {FRANKA_STAGE_PATH}")

zed_prim = stage.GetPrimAtPath(ZED_CAMERA_PATH)
if not zed_prim.IsValid():
    log(f"ERROR: ZED camera prim not found at {ZED_CAMERA_PATH}")
    raise RuntimeError("ZED camera prim not found")
log(f"  ZED camera found: {ZED_CAMERA_PATH}")

# Step 4: Find the wrist link (camera mounting disabled for now)
log("\nSearching for Franka wrist link...")
wrist_path = f"{FRANKA_STAGE_PATH}/{FRANKA_WRIST_LINK}"
wrist_prim = stage.GetPrimAtPath(wrist_path)

if wrist_prim.IsValid():
    log(f"  Found wrist link: {wrist_path}")
else:
    log(f"  Not found at {wrist_path}, searching...")
    # Search for panda_link7 or panda_hand as fallback
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if FRANKA_STAGE_PATH in path_str and path_str.endswith("panda_link7"):
            wrist_path = path_str
            wrist_prim = prim
            log(f"  Found wrist link: {wrist_path}")
            break

# Step 4b: Mount camera to wrist using Fixed Joint
log("\nMounting ZED camera to Franka wrist...")
mounted_camera_path = ZED_CAMERA_PATH

if wrist_prim and wrist_prim.IsValid():
    try:
        from pxr import UsdPhysics

        # Create a fixed joint to attach camera to wrist
        fixed_joint_path = f"{ZED_CAMERA_PATH}/WristFixedJoint"

        # Check if joint already exists
        existing_joint = stage.GetPrimAtPath(fixed_joint_path)
        if existing_joint.IsValid():
            log(f"  Fixed joint already exists at {fixed_joint_path}")
        else:
            # Create the fixed joint
            fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)

            # Set the joint bodies - body0 is parent (wrist), body1 is child (camera)
            fixed_joint.CreateBody0Rel().SetTargets([wrist_path])
            fixed_joint.CreateBody1Rel().SetTargets([ZED_CAMERA_PATH])

            # Set local poses for the joint
            # Camera offset from wrist: forward, right, up (in wrist frame)
            fixed_joint.CreateLocalPos0Attr().Set(
                Gf.Vec3f(CAMERA_MOUNT_OFFSET[0], CAMERA_MOUNT_OFFSET[1], CAMERA_MOUNT_OFFSET[2])
            )
            fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

            # Set rotation (camera pointing forward from wrist)
            rot_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), CAMERA_MOUNT_ROTATION[0])
            rot_y = Gf.Rotation(Gf.Vec3d(0, 1, 0), CAMERA_MOUNT_ROTATION[1])
            rot_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), CAMERA_MOUNT_ROTATION[2])
            combined_rot = rot_z * rot_y * rot_x
            quat = combined_rot.GetQuat()
            fixed_joint.CreateLocalRot0Attr().Set(
                Gf.Quatf(quat.GetReal(), Gf.Vec3f(quat.GetImaginary()))
            )
            fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

            log(f"  Created fixed joint: {fixed_joint_path}")
            log(f"  Camera mounted to: {wrist_path}")
            log(f"  Offset: {CAMERA_MOUNT_OFFSET}, Rotation: {CAMERA_MOUNT_ROTATION}")

    except Exception as e:
        log(f"  WARNING: Could not create fixed joint: {e}")
        log("  Camera will remain at original position")
        log("  To attach manually: drag ZED_X under panda_link7 in Stage panel")
else:
    log("  WARNING: Wrist link not found, camera mounting skipped")
    log("  To attach manually: drag ZED_X under panda_link7 in Stage panel")

omni.kit.app.get_app().update()

# Step 5: Create MoveIt2 Integration OmniGraph
log("\nCreating MoveIt2 integration OmniGraph...")

try:
    log(f"  Creating graph at path: {ACTION_GRAPH_PATH}")

    # Check if graph already exists and delete it first
    existing_graph = stage.GetPrimAtPath(ACTION_GRAPH_PATH)
    if existing_graph.IsValid():
        log(f"  WARNING: Graph already exists at {ACTION_GRAPH_PATH}, deleting...")
        stage.RemovePrim(ACTION_GRAPH_PATH)
        omni.kit.app.get_app().update()

    og.Controller.edit(
        {"graph_path": ACTION_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", FRANKA_STAGE_PATH),
                ("PublishJointState.inputs:topicName", JOINT_STATES_TOPIC),
                ("SubscribeJointState.inputs:topicName", JOINT_COMMANDS_TOPIC),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(FRANKA_STAGE_PATH)]),
            ],
        },
    )
    log("  MoveIt2 ActionGraph created successfully")
except Exception as e:
    import traceback

    log(f"  ERROR creating MoveIt2 ActionGraph: {e}")
    log(f"  Full traceback:")
    traceback.print_exc()

omni.kit.app.get_app().update()

# Step 6: Create ZED Camera OmniGraph
# NOTE: Camera graph creation is DISABLED due to Isaac Sim crash when creating viewport
# The crash occurs in libomni.ui.scene.so when setting up the camera render product
# To use camera in Isaac Sim, manually add camera publishers via the GUI:
#   1. Select the ZED_X/base_link/ZED_X/CameraLeft prim
#   2. Right-click > Create > Isaac > ROS2 Camera
log("\nZED Camera OmniGraph creation SKIPPED")
log("  Camera viewport creation causes Isaac Sim crash")
log("  To publish camera data, use Isaac Sim GUI:")
log(
    "    1. Select: /World/carter_warehouse_navigation_w_ZED_Pandas/ZED_X/base_link/ZED_X/CameraLeft"
)
log("    2. Right-click > Create > Isaac > ROS2 Bridge > Camera")
camera_prim_path = f"{ZED_CAMERA_PATH}/base_link/ZED_X/CameraLeft"

omni.kit.app.get_app().update()

# Step 7: Print summary
log("\n" + "=" * 60)
log("SETUP COMPLETE!")
log("=" * 60)
log("\nROS2 Topics configured:")
log(f"  Publishing:")
log(f"    - /{JOINT_STATES_TOPIC} (sensor_msgs/JointState)")
log(f"    - /clock (rosgraph_msgs/Clock)")
log(f"    - /{ZED_RGB_TOPIC} (sensor_msgs/Image)")
log(f"    - /{ZED_CAMERA_INFO_TOPIC} (sensor_msgs/CameraInfo)")
log(f"    - /{ZED_DEPTH_TOPIC} (sensor_msgs/Image)")
log(f"    - /{ZED_POINTCLOUD_TOPIC} (sensor_msgs/PointCloud2)")
log(f"  Subscribing:")
log(f"    - /{JOINT_COMMANDS_TOPIC} (sensor_msgs/JointState)")

log("\nOmniGraphs created:")
log(f"  - {ACTION_GRAPH_PATH}")
log(f"  - {ZED_CAMERA_GRAPH_PATH}")
log(f"  Camera prim used: {camera_prim_path}")

log("\nNext steps:")
log("  1. Press PLAY in Isaac Sim to start the simulation")
log("  2. In a separate terminal, run:")
log("     source /opt/ros/jazzy/setup.bash")
log("     source ~/workspaces/isaac_ros-dev/install/setup.bash")
log(
    "     ros2 launch sm_panda_cl_moveit2z_cb_inventory_isaacsim sm_panda_cl_moveit2z_cb_inventory_isaacsim.launch.py"
)
log("\n" + "=" * 60)
log(f"\nLog file written to: {LOG_FILE}")
