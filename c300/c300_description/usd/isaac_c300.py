# -*- coding: utf-8 -*-

import argparse
import os
import sys

import carb
import numpy as np
from omni.isaac.kit import SimulationApp

C300_STAGE_PATH = "/c300"
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = (
    "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
)

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
    C300_USD_PATH = args.Path + "/c300.usd"
    # BACKGROUND_USD_PATH = args.Path + "/simple_room.usd"
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
    usd_path=C300_USD_PATH,
)

simulation_app.update()

# Creating a action graph to publish the ROS /clock topic
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )
except Exception as e:
    print(e)

simulation_app.update()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()

while simulation_app.is_running():
    # run with a realtime clock
    simulation_app.update()

simulation_context.stop()
simulation_app.close()
