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

# Franka wrist link for camera mounting (panda_link7 is the last arm link before panda_hand)
FRANKA_WRIST_LINK = "panda_link7"

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

print("=" * 60)
print("Isaac Sim ROS2 Bridge Setup Script")
print("=" * 60)

# Step 1: Get the current stage
stage = omni.usd.get_context().get_stage()
if stage is None:
    print("ERROR: No stage is currently open!")
    print("Please open a USD stage first (File > Open)")
    raise RuntimeError("No stage open")

print(f"Stage loaded: {stage.GetRootLayer().identifier}")

# Step 2: Enable ROS2 bridge extension
print("\nEnabling ROS2 bridge extension...")
enable_extension("isaacsim.ros2.bridge")
omni.kit.app.get_app().update()
print("ROS2 bridge extension enabled")

# Step 3: Verify prims exist
print("\nVerifying prim paths...")
franka_prim = stage.GetPrimAtPath(FRANKA_STAGE_PATH)
if not franka_prim.IsValid():
    print(f"ERROR: Franka prim not found at {FRANKA_STAGE_PATH}")
    print("\nSearching for Franka/Panda prims in stage...")
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if "panda" in path_str.lower() or "franka" in path_str.lower():
            # Show robot prims up to 5 levels deep
            if path_str.count("/") <= 5 and (
                "instanceable" in path_str.lower() or path_str.endswith("panda")
            ):
                print(f"  Found: {path_str} (type: {prim.GetTypeName()})")
    print("\nAll top-level prims under /World:")
    world_prim = stage.GetPrimAtPath("/World")
    if world_prim.IsValid():
        for child in world_prim.GetChildren():
            print(f"  /World/{child.GetName()}")
    raise RuntimeError("Franka prim not found - check paths above and update FRANKA_STAGE_PATH")
print(f"  Franka found: {FRANKA_STAGE_PATH}")

zed_prim = stage.GetPrimAtPath(ZED_CAMERA_PATH)
if not zed_prim.IsValid():
    print(f"ERROR: ZED camera prim not found at {ZED_CAMERA_PATH}")
    raise RuntimeError("ZED camera prim not found")
print(f"  ZED camera found: {ZED_CAMERA_PATH}")

# Step 4: Find the wrist link (camera mounting disabled for now)
print("\nSearching for Franka wrist link...")
wrist_path = f"{FRANKA_STAGE_PATH}/{FRANKA_WRIST_LINK}"
wrist_prim = stage.GetPrimAtPath(wrist_path)

if wrist_prim.IsValid():
    print(f"  Found wrist link: {wrist_path}")
else:
    print(f"  Not found at {wrist_path}, searching...")
    # Search for panda_link7 or panda_hand as fallback
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if FRANKA_STAGE_PATH in path_str and path_str.endswith("panda_link7"):
            wrist_path = path_str
            wrist_prim = prim
            print(f"  Found wrist link: {wrist_path}")
            break

# Camera mounting is disabled - the ZED X has incompatible transform ops
# To manually attach the camera:
# 1. In Isaac Sim Stage panel, drag ZED_X under panda_link7
# 2. Or create a Fixed Joint manually in the Physics panel
print("\nNOTE: Camera mounting skipped (transform rotation order mismatch)")
print("  To attach camera manually: drag ZED_X under panda_link7 in Stage panel")
mounted_camera_path = ZED_CAMERA_PATH

omni.kit.app.get_app().update()

# Step 5: Create MoveIt2 Integration OmniGraph
print("\nCreating MoveIt2 integration OmniGraph...")

try:
    print(f"  Creating graph at path: {ACTION_GRAPH_PATH}")

    # Check if graph already exists and delete it first
    existing_graph = stage.GetPrimAtPath(ACTION_GRAPH_PATH)
    if existing_graph.IsValid():
        print(f"  WARNING: Graph already exists at {ACTION_GRAPH_PATH}, deleting...")
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
    print("  MoveIt2 ActionGraph created successfully")
except Exception as e:
    import traceback

    print(f"  ERROR creating MoveIt2 ActionGraph: {e}")
    print(f"  Full traceback:")
    traceback.print_exc()

omni.kit.app.get_app().update()

# Step 6: Create ZED Camera OmniGraph
print("\nCreating ZED Camera OmniGraph...")
print(f"  Creating graph at path: {ZED_CAMERA_GRAPH_PATH}")

# Check if graph already exists and delete it first
existing_zed_graph = stage.GetPrimAtPath(ZED_CAMERA_GRAPH_PATH)
if existing_zed_graph.IsValid():
    print(f"  WARNING: Graph already exists at {ZED_CAMERA_GRAPH_PATH}, deleting...")
    stage.RemovePrim(ZED_CAMERA_GRAPH_PATH)
    omni.kit.app.get_app().update()

# Find the camera prim inside the ZED_X
camera_prim_path = mounted_camera_path
# Look for actual Camera prim inside ZED_X
zed_prim_check = stage.GetPrimAtPath(mounted_camera_path)
if zed_prim_check.IsValid():
    for child in zed_prim_check.GetChildren():
        if child.GetTypeName() == "Camera":
            camera_prim_path = str(child.GetPath())
            print(f"  Found camera prim: {camera_prim_path}")
            break
else:
    print(f"  WARNING: ZED prim not found at {mounted_camera_path}")

try:
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": ZED_CAMERA_GRAPH_PATH,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperPointCloud", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                (
                    "getRenderProduct.outputs:renderProductPath",
                    "setCamera.inputs:renderProductPath",
                ),
                ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperPointCloud.inputs:execIn"),
                (
                    "getRenderProduct.outputs:renderProductPath",
                    "cameraHelperRgb.inputs:renderProductPath",
                ),
                (
                    "getRenderProduct.outputs:renderProductPath",
                    "cameraHelperInfo.inputs:renderProductPath",
                ),
                (
                    "getRenderProduct.outputs:renderProductPath",
                    "cameraHelperDepth.inputs:renderProductPath",
                ),
                (
                    "getRenderProduct.outputs:renderProductPath",
                    "cameraHelperPointCloud.inputs:renderProductPath",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("createViewport.inputs:viewportId", 1),
                ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                ("cameraHelperRgb.inputs:topicName", ZED_RGB_TOPIC),
                ("cameraHelperRgb.inputs:frameId", ZED_FRAME_ID),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("cameraHelperInfo.inputs:topicName", ZED_CAMERA_INFO_TOPIC),
                ("cameraHelperInfo.inputs:frameId", ZED_FRAME_ID),
                ("cameraHelperDepth.inputs:topicName", ZED_DEPTH_TOPIC),
                ("cameraHelperDepth.inputs:frameId", ZED_FRAME_ID),
                ("cameraHelperDepth.inputs:type", "depth"),
                ("cameraHelperPointCloud.inputs:topicName", ZED_POINTCLOUD_TOPIC),
                ("cameraHelperPointCloud.inputs:frameId", ZED_FRAME_ID),
                ("cameraHelperPointCloud.inputs:type", "depth_pcl"),
            ],
        },
    )

    # Evaluate once to initialize the graph
    og.Controller.evaluate_sync(ros_camera_graph)
    print("  ZED Camera OmniGraph created successfully")

except Exception as e:
    import traceback

    print(f"  ERROR creating ZED Camera OmniGraph: {e}")
    print(f"  Full traceback:")
    traceback.print_exc()

omni.kit.app.get_app().update()

# Step 7: Print summary
print("\n" + "=" * 60)
print("SETUP COMPLETE!")
print("=" * 60)
print("\nROS2 Topics configured:")
print(f"  Publishing:")
print(f"    - /{JOINT_STATES_TOPIC} (sensor_msgs/JointState)")
print(f"    - /clock (rosgraph_msgs/Clock)")
print(f"    - /{ZED_RGB_TOPIC} (sensor_msgs/Image)")
print(f"    - /{ZED_CAMERA_INFO_TOPIC} (sensor_msgs/CameraInfo)")
print(f"    - /{ZED_DEPTH_TOPIC} (sensor_msgs/Image)")
print(f"    - /{ZED_POINTCLOUD_TOPIC} (sensor_msgs/PointCloud2)")
print(f"  Subscribing:")
print(f"    - /{JOINT_COMMANDS_TOPIC} (sensor_msgs/JointState)")

print("\nOmniGraphs created:")
print(f"  - {ACTION_GRAPH_PATH}")
print(f"  - {ZED_CAMERA_GRAPH_PATH}")

print("\nNext steps:")
print("  1. Press PLAY in Isaac Sim to start the simulation")
print("  2. In a separate terminal, run:")
print("     source /opt/ros/jazzy/setup.bash")
print("     source ~/workspaces/isaac_ros-dev/install/setup.bash")
print(
    "     ros2 launch sm_panda_cl_moveit2z_cb_inventory_isaacsim sm_panda_cl_moveit2z_cb_inventory_isaacsim.launch.py"
)
print("\n" + "=" * 60)

# Debug: Print panda structure if verbose
print("\nDebug: Searching for panda links...")
for prim in stage.Traverse():
    path_str = str(prim.GetPath())
    if "panda" in path_str.lower() and ("link" in path_str.lower() or "joint" in path_str.lower()):
        print(f"  {path_str}")
