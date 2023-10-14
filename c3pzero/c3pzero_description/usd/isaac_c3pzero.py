# -*- coding: utf-8 -*-

import argparse
import os
import sys

import carb
import numpy as np
from omni.isaac.kit import SimulationApp

C3PZERO_STAGE_PATH = "/c3pzero"
CAMERA_PRIM_PATH = (
    f"{C3PZERO_STAGE_PATH}/kbot/wrist_mounted_camera_color_frame/RealsenseCamera"
)
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = (
    "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
)
REALSENSE_VIEWPORT_NAME = "realsense_viewport"

# Initialize the parser
parser = argparse.ArgumentParser(
    description="Process the path to the robot's folder containing its UDS file"
)

# Add the arguments
parser.add_argument(
    "Path", metavar="path", type=str, help="the path to the robot's folder"
)

# Parse the arguments
args = parser.parse_args()

# Check if the path argument was provided
if args.Path:
    GEN3_USD_PATH = args.Path + "/c3pzero_composite.usd"
else:
    print(
        "[ERROR] This script requires an argument with the absolute path to the robot's folder containing it's UDS file"
    )
    sys.exit()

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of stages
# and creation of ROS components
simulation_app = SimulationApp(CONFIG)

import omni.graph.core as og  # noqa E402
from omni.isaac.core import SimulationContext  # noqa E402
from omni.isaac.core.utils import (  # noqa E402
    extensions,
    nucleus,
    prims,
    rotations,
    stage,
    viewports,
)
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
from pxr import Gf, UsdGeom  # noqa E402
from pxr import Gf  # noqa E402
import omni.ui  # to dock realsense viewport automatically

# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Locate Isaac Sim assets folder to load environment and robot stages
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Preparing stage
viewports.set_camera_view(eye=np.array([3, 0.5, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the simple_room environment
stage.add_reference_to_stage(
    assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
)

# Loading the gen3 robot USD
prims.create_prim(
    C3PZERO_STAGE_PATH,
    "Xform",
    position=np.array([1, 0, 0.17]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=GEN3_USD_PATH,
)

simulation_app.update()

# Creating a action graph with ROS component nodes
# TODO: Creating the omnigraph here is getting ridiculous!
#       Move them into the USD files or refactor to use helper functions to build the graph.
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                (
                    "MobileBaseSubscribeJointState",
                    "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                ),
                (
                    "MobileBaseArticulationController",
                    "omni.isaac.core_nodes.IsaacArticulationController",
                ),
                (
                    "ManipulatorSubscribeJointState",
                    "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                ),
                (
                    "ManipulatorArticulationController",
                    "omni.isaac.core_nodes.IsaacArticulationController",
                ),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("IsaacReadLidarBeams", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                ("PublishLidarScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
                # Nodes to subtract some time of the lidar message so it's timestamps match the tf tree in ROS
                ("ConstantFloat", "omni.graph.nodes.ConstantFloat"),
                ("Subtract", "omni.graph.nodes.Subtract"),
                # Wrist camera
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                (
                    "getRenderProduct",
                    "omni.isaac.core_nodes.IsaacGetViewportRenderProduct",
                ),
                ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "MobileBaseSubscribeJointState.inputs:execIn",
                ),
                (
                    "Context.outputs:context",
                    "MobileBaseSubscribeJointState.inputs:context",
                ),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "MobileBaseArticulationController.inputs:execIn",
                ),
                (
                    "MobileBaseSubscribeJointState.outputs:jointNames",
                    "MobileBaseArticulationController.inputs:jointNames",
                ),
                (
                    "MobileBaseSubscribeJointState.outputs:positionCommand",
                    "MobileBaseArticulationController.inputs:positionCommand",
                ),
                (
                    "MobileBaseSubscribeJointState.outputs:velocityCommand",
                    "MobileBaseArticulationController.inputs:velocityCommand",
                ),
                (
                    "MobileBaseSubscribeJointState.outputs:effortCommand",
                    "MobileBaseArticulationController.inputs:effortCommand",
                ),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "ManipulatorSubscribeJointState.inputs:execIn",
                ),
                (
                    "Context.outputs:context",
                    "ManipulatorSubscribeJointState.inputs:context",
                ),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "ManipulatorArticulationController.inputs:execIn",
                ),
                (
                    "ManipulatorSubscribeJointState.outputs:jointNames",
                    "ManipulatorArticulationController.inputs:jointNames",
                ),
                (
                    "ManipulatorSubscribeJointState.outputs:positionCommand",
                    "ManipulatorArticulationController.inputs:positionCommand",
                ),
                (
                    "ManipulatorSubscribeJointState.outputs:velocityCommand",
                    "ManipulatorArticulationController.inputs:velocityCommand",
                ),
                (
                    "ManipulatorSubscribeJointState.outputs:effortCommand",
                    "ManipulatorArticulationController.inputs:effortCommand",
                ),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishJointState.inputs:timeStamp",
                ),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                # Hack time offset for lidar messages
                ("ReadSimTime.outputs:simulationTime", "Subtract.inputs:a"),
                ("ConstantFloat.inputs:value", "Subtract.inputs:b"),
                # Lidar nodes
                (
                    "OnImpulseEvent.outputs:execOut",
                    "IsaacReadLidarBeams.inputs:execIn",
                ),
                (
                    "IsaacReadLidarBeams.outputs:execOut",
                    "PublishLidarScan.inputs:execIn",
                ),
                (
                    "IsaacReadLidarBeams.outputs:azimuthRange",
                    "PublishLidarScan.inputs:azimuthRange",
                ),
                (
                    "IsaacReadLidarBeams.outputs:depthRange",
                    "PublishLidarScan.inputs:depthRange",
                ),
                (
                    "IsaacReadLidarBeams.outputs:horizontalFov",
                    "PublishLidarScan.inputs:horizontalFov",
                ),
                (
                    "IsaacReadLidarBeams.outputs:horizontalResolution",
                    "PublishLidarScan.inputs:horizontalResolution",
                ),
                (
                    "IsaacReadLidarBeams.outputs:intensitiesData",
                    "PublishLidarScan.inputs:intensitiesData",
                ),
                (
                    "IsaacReadLidarBeams.outputs:linearDepthData",
                    "PublishLidarScan.inputs:linearDepthData",
                ),
                (
                    "IsaacReadLidarBeams.outputs:numCols",
                    "PublishLidarScan.inputs:numCols",
                ),
                (
                    "IsaacReadLidarBeams.outputs:numRows",
                    "PublishLidarScan.inputs:numRows",
                ),
                (
                    "IsaacReadLidarBeams.outputs:rotationRate",
                    "PublishLidarScan.inputs:rotationRate",
                ),
                (
                    "Subtract.outputs:difference",
                    "PublishLidarScan.inputs:timeStamp",
                ),
                ("Context.outputs:context", "PublishLidarScan.inputs:context"),
                # wrist camera
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
                ("Context.outputs:context", "cameraHelperRgb.inputs:context"),
                ("Context.outputs:context", "cameraHelperInfo.inputs:context"),
                ("Context.outputs:context", "cameraHelperDepth.inputs:context"),
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
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", int(os.environ["ROS_DOMAIN_ID"])),
                # Setting the /c3pzero target prim to Articulation Controller node
                ("MobileBaseArticulationController.inputs:usePath", True),
                (
                    "MobileBaseArticulationController.inputs:robotPath",
                    C3PZERO_STAGE_PATH,
                ),
                (
                    "MobileBaseSubscribeJointState.inputs:topicName",
                    "mobile_base_joint_commands",
                ),
                ("ManipulatorArticulationController.inputs:usePath", True),
                (
                    "ManipulatorArticulationController.inputs:robotPath",
                    C3PZERO_STAGE_PATH,
                ),
                (
                    "ManipulatorSubscribeJointState.inputs:topicName",
                    "manipulator_joint_commands",
                ),
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                (
                    "PublishLidarScan.inputs:frameId",
                    "base_laser",
                ),  # c300's laser frame_id
                ("PublishLidarScan.inputs:topicName", "scan"),
                # Hack time offset for lidar messages
                ("ConstantFloat.inputs:value", 0.1),
                # Wrist camera
                ("createViewport.inputs:name", REALSENSE_VIEWPORT_NAME),
                ("createViewport.inputs:viewportId", 1),
                (
                    "cameraHelperRgb.inputs:frameId",
                    "wrist_mounted_camera_color_optical_frame",
                ),
                (
                    "cameraHelperRgb.inputs:topicName",
                    "/wrist_mounted_camera/color/image_raw",
                ),
                ("cameraHelperRgb.inputs:type", "rgb"),
                (
                    "cameraHelperInfo.inputs:frameId",
                    "wrist_mounted_camera_color_optical_frame",
                ),
                (
                    "cameraHelperInfo.inputs:topicName",
                    "/wrist_mounted_camera/color/camera_info",
                ),
                ("cameraHelperInfo.inputs:type", "camera_info"),
                (
                    "cameraHelperDepth.inputs:frameId",
                    "wrist_mounted_camera_color_optical_frame",
                ),
                (
                    "cameraHelperDepth.inputs:topicName",
                    "/wrist_mounted_camera/depth/image_rect_raw",
                ),
                ("cameraHelperDepth.inputs:type", "depth"),
            ],
        },
    )
except Exception as e:
    print(e)


# Setting the /c300 target prim to Publish JointState node
set_target_prims(
    primPath="/ActionGraph/PublishJointState", targetPrimPaths=[C3PZERO_STAGE_PATH]
)
# Setting the /c300's Lidar target prim to read scan data in the simulation
set_target_prims(
    primPath="/ActionGraph/IsaacReadLidarBeams",
    inputName="inputs:lidarPrim",
    targetPrimPaths=[
        C3PZERO_STAGE_PATH + "/c300/Chassis/Laser/UAM_05LP/UAM_05LP/Scan/Lidar"
    ],
)

# Fix camera settings since the defaults in the realsense model are inaccurate
realsense_prim = UsdGeom.Camera(
    stage.get_current_stage().GetPrimAtPath(CAMERA_PRIM_PATH)
)
realsense_prim.GetHorizontalApertureAttr().Set(20.955)
realsense_prim.GetVerticalApertureAttr().Set(11.7)
realsense_prim.GetFocalLengthAttr().Set(18.8)
realsense_prim.GetFocusDistanceAttr().Set(400)
realsense_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000000.0))

set_targets(
    prim=stage.get_current_stage().GetPrimAtPath("/ActionGraph/setCamera"),
    attribute="inputs:cameraPrim",
    target_prim_paths=[CAMERA_PRIM_PATH],
)

simulation_app.update()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()

# Dock the second camera window
viewport = omni.ui.Workspace.get_window("Viewport")
rs_viewport = omni.ui.Workspace.get_window(REALSENSE_VIEWPORT_NAME)
rs_viewport.dock_in(viewport, omni.ui.DockPosition.RIGHT)

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    og.Controller.set(
        og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    )

simulation_context.stop()
simulation_app.close()
