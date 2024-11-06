from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units, print_stage_prim_paths
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from test.robot import HuskyFR3Robot, JOINT_NAMES
from husky_fr3_controller import AggregationController
from stable_baselines_example.differential_controller import DifferentialController
import numpy as np
import carb

from typing import Union
from omni.isaac.core.objects import DynamicCuboid
import matplotlib.pyplot as plt

np.set_printoptions(formatter={'float_kind': lambda x: f"{x:0.3e}"})

my_world = World(stage_units_in_meters=1.0)
asset_path = "/home/bluesun/Desktop/Husky/husky_fr3/world4.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/husky")
my_husky: HuskyFR3Robot = \
    my_world.scene.add(HuskyFR3Robot(prim_path="/World/husky",
                                     name="my_husky",
                                     create_robot=False,
                                     usd_path=asset_path,
                                     position=np.array([0, 0.0, 0.2]),
                                     orientation=np.array([1, -1, 0, 0]), ))

cube_prim_path = find_unique_string_name(initial_name="/World/Cube", is_unique_fn=lambda x: not is_prim_path_valid(x))
cube_name = find_unique_string_name(initial_name="cube", is_unique_fn=lambda x: not my_world.scene.object_exists(x))
my_cube: DynamicCuboid = my_world.scene.add(DynamicCuboid(name=cube_name,
                                                          position=np.array([1.2, 0.3, 0.3]) / get_stage_units(),
                                                          orientation=np.array([1, 0, 0, 0]),
                                                          prim_path=cube_prim_path,
                                                          scale=np.array([0.0515, 0.0515, 0.0515]) / get_stage_units(),
                                                          size=1.0,
                                                          color=np.array([0, 0, 1])))

wheel_controller = DifferentialController(name="simple_control", wheel_radius=0.3, wheel_base=0.1125)
franka_controller = PickPlaceController(name="pick_place_controller",
                                        gripper=my_husky.gripper,
                                        robot_articulation=my_husky)
husky_controller = AggregationController(robot_articulation=my_husky)
my_world.scene.add_default_ground_plane()
my_world.reset()


def quaternion_to_euler(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.math.degrees(np.math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = np.math.degrees(np.math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.math.degrees(np.math.atan2(t3, t4))

    return np.array([X, Y, Z])

def calc_target_pos():
    husky_pos, husky_dir = my_husky.get_local_pose()
    husky_dir = np.cos(np.deg2rad(quaternion_to_euler(*husky_dir)))
    husky_dir = husky_dir / np.linalg.norm(husky_dir)
    angle = np.deg2rad(20)
    rot = np.array([[np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0, 0, 1]])
    target_pos = husky_pos + np.matmul(husky_dir, rot) * 0.5
    target_pos[2] = my_cube.get_size() / 2.0
    return target_pos


i = 0

pos_log = []
vel_log = []
pos_action_log = []
vel_action_log = []
dof_log = []

for _ in range(20):
    my_world.step(render=True)
    pos_log.append(my_husky.get_joint_positions())
    vel_log.append(my_husky.get_joint_velocities())
    applied_action = my_husky.get_applied_action()
    pos_action_log.append(applied_action.joint_positions)
    vel_action_log.append(applied_action.joint_velocities)

for _ in range(100):
    my_world.step(render=True)

action_log = np.load('follow_target.npz')
pos_actions = action_log['pos_log']
vel_actions = action_log['vel_log']
action_selector = zip(pos_actions, vel_actions)

joint_pos_action = np.zeros(13)
joint_vel_action = np.zeros(13)

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            husky_controller.reset()
            wheel_controller.reset()
            franka_controller.reset()
            i = 0

        sampled_action = next(action_selector, None)
        if sampled_action is not None:
            pos_log.append(my_husky.get_joint_positions())
            vel_log.append(my_husky.get_joint_velocities())
            dof_log.append(my_husky.get_local_pose()[0].sum())
            applied_action = my_husky.get_applied_action()
            pos_action_log.append(applied_action.joint_positions)
            vel_action_log.append(applied_action.joint_velocities)

            # joint_pos_action[i//step_size] = -np.pi / 4 * (i % step_size) / step_size
            joint_vel_action = np.zeros(13)
            joint_vel_action[:7] = sampled_action[1][:7]
            joint_pos_action = applied_action.joint_positions
            joint_pos_action[:7] = sampled_action[0][:7]
            # actions = ArticulationAction(joint_positions=joint_pos_action, joint_velocities=joint_vel_action)
            # my_husky._articulation_view.set_joint_positions(joint_pos_action)
            my_husky._articulation_view.set_joint_velocities(joint_vel_action)
            my_husky._articulation_view.set_joint_positions(joint_pos_action)
            # articulation_controller.apply_action(actions)
        else:
            break
        i += 1
        #
        # observations = my_world.get_observations()
        # actions = my_controller.forward(
        #     target_end_effector_position=observations[target_name]["position"],
        #     target_end_effector_orientation=observations[target_name]["orientation"],
        # )
        # articulation_controller.apply_action(actions)

pos_log = np.array(pos_log)
vel_log = np.array(vel_log)
pos_action_log = np.array(pos_action_log)
vel_action_log = np.array(vel_action_log)

fig, axes = plt.subplots(4, 1, figsize=(10, 20), dpi=150)
for k in range(pos_log.shape[-1]):
    c = plt.cm.tab20c.colors[k]
    axes[0].plot(pos_log[:, k], label=str(k), c=c)
    axes[1].plot(vel_log[:, k], label=str(k), c=c)
    axes[2].plot(pos_action_log[:, k], label=str(k), c=c)
    axes[3].plot(vel_action_log[:, k], label=str(k), c=c)

axes[2].plot(np.array(dof_log), label='dof', c='k')

for ax in axes:
    ax.legend()

axes[0].set_title("Position")
axes[0].set_ylim((-1, 1))
axes[1].set_title("Velocity")
axes[1].set_ylim((-1, 1))
axes[2].set_title("Position Action")
axes[2].set_ylim((-1, 1))
axes[3].set_title("Velocity Action")
axes[3].set_ylim((-1, 1))

plt.savefig('move5.png')

simulation_app.close()
