# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.robots import WheeledRobot
# from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from stable_baselines_example.differential_controller import DifferentialController
import numpy as np
import carb
from omni.isaac.core.utils.types import ArticulationAction

my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
jetbot_asset_path = "/home/bluesun/Desktop/Husky/husky_fr3/husky/husky.usd"
my_jetbot: WheeledRobot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/husky",
        name="my_jetbot",
        wheel_dof_names=["front_left_wheel",
                         "front_right_wheel",
                         "rear_left_wheel",
                         "rear_right_wheel"],
        wheel_dof_indices=[0, 1, 2, 3],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([0, 0.0, 2.0]),
    )
)
my_world.scene.add_default_ground_plane()
my_controller = DifferentialController(name="simple_control", wheel_radius=0.3, wheel_base=0.1125)
my_world.reset()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()

    my_jetbot.apply_wheel_actions(my_controller.forward(command=np.array([0.5, 0])))
    print('forward: ' + str([f'{v:.4e}' for v in my_jetbot.get_linear_velocity()]))
    print(f'pos: {my_jetbot.get_world_pose()}')

    if args.test is True:
        break

simulation_app.close()
