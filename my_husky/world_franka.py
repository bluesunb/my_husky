from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units, print_stage_prim_paths
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

# from husky_robot import MyRobot, JOINT_NAMES
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
import carb

from typing import Union
from omni.isaac.core.objects import DynamicCuboid
import matplotlib.pyplot as plt

np.set_printoptions(precision=3, suppress=False)

my_world = World(stage_units_in_meters=1.0)
# asset_path = "/home/bluesun/Desktop/Husky/husky_fr3/world4.usd"
# add_reference_to_stage(usd_path=asset_path, prim_path="/World/husky")
my_franak: Franka = \
    my_world.scene.add(Franka(prim_path="/World/franka",
                              name="my_franka")
                       )

cube_prim_path = find_unique_string_name(initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x))
cube_name = find_unique_string_name(initial_name="cube", is_unique_fn=lambda x: not my_world.scene.object_exists(x))
my_cube: DynamicCuboid = my_world.scene.add(DynamicCuboid(name=cube_name,
                                                          position=np.array([1.2, 0.3, 0.3]) / get_stage_units(),
                                                          orientation=np.array([1, 0, 0, 0]),
                                                          prim_path=cube_prim_path,
                                                          scale=np.array([0.0515, 0.0515, 0.0515]) / get_stage_units(),
                                                          size=1.0,
                                                          color=np.array([0, 0, 1])))

articulation_controller = my_franak.get_articulation_controller()
my_world.scene.add_default_ground_plane()
my_world.reset()
i = 0
def step(n: int):
    for i in range(n):
        my_world.step(render=True)

while simulation_app.is_running():
    my_world.step(render=True)
    # print(my_husky.get_local_pose())
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            i = 0

        dof_names = my_franak.dof_names
        dof_prop = my_franak.dof_properties
        dof_dict = {name: my_franak.get_dof_index(name) for name in dof_names}

        loc_pos = my_franak.get_local_pose()[0]
        loc_ori = my_franak.get_local_pose()[1]
        loc_ori_euler = quat_to_euler_angles(loc_ori)
        loc_vel = my_franak.get_linear_velocity()

        joint_pos = my_franak.get_joint_positions()
        joint_vel = my_franak.get_joint_velocities()

        applied_action = my_franak.get_applied_action()
        pos_action = applied_action.joint_positions
        vel_action = applied_action.joint_velocities

        if i == 250:
            my_franak.set_joint_positions(np.random.rand(9) * 2 - 1)

        i = (i + 1)%500

simulation_app.close()
