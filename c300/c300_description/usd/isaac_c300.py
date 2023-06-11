# -*- coding: utf-8 -*-


import os
import sys

import carb
import numpy as np
from omni.isaac.kit import SimulationApp

C300_STAGE_PATH = "/c300"
if len(sys.argv) <= 1:
    print(
        "[ERROR] This script requires an argument with the absolute path to the robot's folder containing it's UDS file"
    )
    sys.exit()
GEN3_USD_PATH = sys.argv[1] + "/c300.usd"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = (
    "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
)

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
from omni.isaac.core_nodes.scripts.utils import set_target_prims  # noqa E402
from pxr import Gf  # noqa E402

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
    C300_STAGE_PATH,
    "Xform",
    position=np.array([1, 0, 0.17]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=GEN3_USD_PATH,
)

simulation_app.update()

# Creating a action graph with ROS component nodes
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
                    "SubscribeJointState",
                    "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                ),
                (
                    "ArticulationController",
                    "omni.isaac.core_nodes.IsaacArticulationController",
                ),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("IsaacReadLidarBeams", "omni.isaac.range_sensor.IsaacReadLidarBeams"),
                ("PublishLidarScan", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                (
                    "OnImpulseEvent.outputs:execOut",
                    "ArticulationController.inputs:execIn",
                ),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishJointState.inputs:timeStamp",
                ),
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
                    "ReadSimTime.outputs:simulationTime",
                    "PublishLidarScan.inputs:timeStamp",
                ),
                ("Context.outputs:context", "PublishLidarScan.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Context.inputs:domain_id", 0),  # int(os.environ["ROS_DOMAIN_ID"])),
                # Setting the /c300 target prim to Articulation Controller node
                ("ArticulationController.inputs:usePath", True),
                ("ArticulationController.inputs:robotPath", C300_STAGE_PATH),
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
                (
                    "PublishLidarScan.inputs:frameId",
                    "base_laser",
                ),  # c300's laser frame_id
                ("PublishLidarScan.inputs:topicName", "scan"),
            ],
        },
    )
except Exception as e:
    print(e)


# Setting the /c300 target prim to Publish JointState node
set_target_prims(
    primPath="/ActionGraph/PublishJointState", targetPrimPaths=[C300_STAGE_PATH]
)
# Setting the /c300's Lidar target prim to read scan data in the simulation
set_target_prims(
    primPath="/ActionGraph/IsaacReadLidarBeams",
    inputName="inputs:lidarPrim",
    targetPrimPaths=[C300_STAGE_PATH + "/Chassis/Laser/UAM_05LP/UAM_05LP/Scan/Lidar"],
)

simulation_app.update()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()

while simulation_app.is_running():

    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
    og.Controller.set(
        og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    )

simulation_context.stop()
simulation_app.close()
