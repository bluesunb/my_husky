from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units, print_stage_prim_paths, get_current_stage
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.prims import is_prim_path_valid, find_matching_prim_paths, get_prim_at_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

from husky_robot import MyRobot, JOINT_NAMES
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np
import carb

from typing import Union, List
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
import matplotlib.pyplot as plt

from utils import smooth_velocity

np.set_printoptions(precision=3, suppress=False)

my_world = World(stage_units_in_meters=1.0)
# asset_path = "/home/bluesun/Desktop/Husky/husky_fr3/world4.usd"
asset_path = "/home/bluesun/Desktop/Husky/husky_fr3_position3.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/husky")
my_husky: MyRobot = \
    my_world.scene.add(MyRobot(prim_path="/World/husky",
                               name="my_husky",
                               usd_path=asset_path,
                               position=np.array([0, 0.0, 0.2]),
                               orientation=np.array([1, -1, 0, 0]), ))

cube_prim_path = find_unique_string_name(initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x))
cube_name = find_unique_string_name(initial_name="cube", is_unique_fn=lambda x: not my_world.scene.object_exists(x))
my_cube: VisualCuboid = my_world.scene.add(VisualCuboid(name=cube_name,
                                                          position=np.array([0.302, -1.158,  0.006]) / get_stage_units(),
                                                          orientation=np.array([1, 0, 0, 0]),
                                                          prim_path=cube_prim_path,
                                                          scale=np.array([0.0515, 0.0515, 0.0515]) / get_stage_units(),
                                                          size=1.0,
                                                          color=np.array([0, 0, 1])))

articulation_controller = my_husky.get_articulation_controller()
my_world.scene.add_default_ground_plane()
my_world.reset()
i = 0
def step(n: int):
    for i in range(n):
        my_world.step(render=True)


# action_log = np.load("../hus_franka/pos_vel_log.npz")
# pos_log = action_log['pos_log']
# vel_log = action_log['vel_log']
# action_iter = zip(pos_log, vel_log)
#

def move_joint(controller_kwargs: dict):
    joint_controllers = {}
    for joint_name, args in controller_kwargs.items():
        init_pos = my_husky.get_joint_positions(my_husky.get_dof_index(joint_name))[0]
        joint_controllers[joint_name] = smooth_velocity(init_pos, *args)

    lens = list(map(len, joint_controllers.values()))
    max_len = max(lens)
    for i in range(max_len):
        joint_vel = np.zeros(13)
        for joint_name, joint_controller in joint_controllers.items():
            idx = my_husky.get_dof_index(joint_name)
            joint_vel[idx] = joint_controller[i] if i < len(joint_controller) else 0

        articulation_controller.apply_action(
            ArticulationAction(joint_velocities=joint_vel)
        )

        # if i % 10 == 0:
            # print(my_husky.get_joint_positions())

        my_world.step(render=True)

joint_limits = {name: articulation_controller.get_joint_limits()[my_husky.get_dof_index(name)]*0.9
                for name in JOINT_NAMES[:7]}

def random_move():
    tgt_poses = {name: (np.random.rand() * (joint_limits[name][1] - joint_limits[name][0]) + joint_limits[name][0],)
                 for name in JOINT_NAMES[:7]}
    print(f'TO: {tgt_poses}')
    move_joint(tgt_poses)


while simulation_app.is_running():
    my_world.step(render=True)
    # print(my_husky.get_local_pose())
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            # action_iter = zip(pos_log, vel_log)
            i = 0

        #### start ####
        dof_names = my_husky.dof_names
        dof_prop = my_husky.dof_properties
        dof_dict = {name: my_husky.get_dof_index(name) for name in dof_names}

        loc_pos = my_husky.get_local_pose()[0]
        loc_ori = my_husky.get_local_pose()[1]
        loc_ori_euler = quat_to_euler_angles(loc_ori)
        loc_vel = my_husky.get_linear_velocity()

        joint_pos = my_husky.get_joint_positions()
        joint_vel = my_husky.get_joint_velocities()

        applied_action = my_husky.get_applied_action()
        pos_action = applied_action.joint_positions
        vel_action = applied_action.joint_velocities

        joint_position_action = np.zeros(13)
        joint_velocity_action = np.zeros(13)

        # move_joint({'fr3_joint2': (np.pi/3,)})
        # move_joint({'fr3_joint4': (-np.pi/2, )})
        # move_joint({'fr3_joint1': (2.3, )})

        dof_indices = [dof_dict[name] for name in JOINT_NAMES]
        joint_velocity_action[-4:] = 0.5
        action = ArticulationAction(joint_velocities=joint_velocity_action, joint_indices=dof_indices)
        my_husky.apply_action(action)

        move_joint({'fr3_joint2': (-np.pi/3, )})
        move_joint({'fr3_joint4': (-np.pi*2/3, )})
        move_joint({'fr3_joint6': (np.pi, )})
        move_joint({'fr3_joint1': (np.pi/4, )})
        move_joint({'fr3_joint3': (np.pi/4, )})
        move_joint({'fr3_joint5': (np.pi/2, )})
        move_joint({'fr3_joint7': (np.pi/4, )})
        step(500)
        move_joint({'fr3_joint1': (0.0, ),
                    'fr3_joint2': (0.0, ),
                    'fr3_joint3': (0.0, ),
                    'fr3_joint4': (-0.5, ),
                    'fr3_joint5': (0.0, ),
                    'fr3_joint6': (0.9, ),
                    'fr3_joint7': (0.0, )})

        i = (i + 1)%100

simulation_app.close()
