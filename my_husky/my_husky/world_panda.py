from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.rotations import quat_to_euler_angles

from husky_panda import MyRobot, JOINT_NAMES
from omni.isaac.core.utils.types import ArticulationAction

import numpy as np

np.set_printoptions(precision=3, suppress=False)


"========== World =========="
my_world = World(stage_units_in_meters=1.0)

asset_path = "/home/bluesun/Desktop/Husky/husky_fr3/my_husky.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/husky")

my_husky: MyRobot = my_world.scene.add(
    MyRobot(prim_path="/World/husky",
            name="my_husky",
            usd_path=asset_path,
            position=np.array([0, 0, 0.2]),
            orientation=np.array([1, 0, 0, 0]))
)

my_world.scene.add_default_ground_plane()
my_world.reset()

"========== Articulation Controller =========="

def step(n: int):
    for i in range(n):
        my_world.step(render=True)

articulation_controller = my_husky.get_articulation_controller()


"========== Simulation =========="

joint_limits = {
    name: articulation_controller.get_joint_limits()[my_husky.get_dof_index(name)]
                for name in JOINT_NAMES[:7]}

default_joints = np.zeros(7)
default_joints[3] = joint_limits['panda_joint4'][1]
default_joints[5] = joint_limits['panda_joint6'][0]

min_joints = np.array([limit[0] for limit in joint_limits.values()])
max_joints = np.array([limit[1] for limit in joint_limits.values()])

delta = default_joints / (max_joints - min_joints) * 2
default_angles = np.arcsin(delta)

i, j = 0, 0

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            default_pos = my_husky.get_joints_default_state()[0]
            my_husky.set_joint_positions(default_pos)

        dof_names = my_husky.dof_names
        dof_prop = my_husky.dof_properties
        dof_dict = {name: my_husky.get_dof_index(name) for name in dof_names}

        loc_pos, loc_ori = my_husky.get_local_pose()
        loc_ori_euler = quat_to_euler_angles(loc_ori)
        loc_vel = my_husky.get_linear_velocity()

        applied_action = my_husky.get_applied_action()
        pos_action = applied_action.joint_velocities
        vel_action = applied_action.joint_velocities

        joint_position_action = np.zeros(my_husky.num_dof)
        joint_velocity_action = np.zeros(my_husky.num_dof)

        y = np.sin(default_angles + i) * (max_joints - min_joints) * 0.4 + \
            (min_joints + max_joints) * 0.5
        j = int(i // np.pi * 2) % 7
        my_husky.set_joint_positions(y[j], [dof_dict[JOINT_NAMES[j]]])

        i += np.pi / 90 * 0.7

simulation_app.close()