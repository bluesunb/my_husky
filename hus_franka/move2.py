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

action_log = np.load('pos_vel_log.npz')
pos_actions = action_log['pos_log']
vel_actions = action_log['vel_log']
action_selector = zip(pos_actions, vel_actions)

joint_pos_action = np.zeros(13)
joint_vel_action = np.zeros(13)

while simulation_app.is_running():
    my_world.step(render=True)
    # print(my_husky.get_local_pose())
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            husky_controller.reset()
            wheel_controller.reset()
            franka_controller.reset()
            i = 0

        # actions = next(action_selector, None)
        # if actions is not None:
        #     joint_pos_action = np.zeros(13)
        #     joint_vel_action = np.zeros(13)
        #     pos_action, vel_action = actions
        #     joint_pos_action[:len(pos_action)] = pos_action
        #     joint_vel_action[:len(vel_action)] = vel_action
        #
        #     my_husky.set_joint_positions(joint_pos_action)
        #     my_husky.set_joint_velocities(joint_vel_action)
        #     pos_log.append(joint_pos_action)
        #     vel_log.append(joint_vel_action)
        #     applied_action = my_husky.get_applied_action()
        #     pos_action_log.append(applied_action.joint_positions)
        #     vel_action_log.append(applied_action.joint_velocities)

        # j = 0
        # joint_idx = my_husky.get_dof_index(JOINT_NAMES[j])
        # init_pos = my_husky.get_joint_positions(joint_idx)
        # target_pos = init_pos + np.pi / 4 / 300
        # target_vel = 0.01
        # for _ in range(200):
        #     my_husky.set_joint_positions(target_pos, joint_idx)
        #     my_husky.set_joint_velocities(target_vel, joint_idx)
        #     my_world.step(render=True)
        #     pos_log.append(my_husky.get_joint_positions())
        #     vel_log.append(my_husky.get_joint_velocities())
        #     applied_action = my_husky.get_applied_action()
        #     pos_action_log.append(applied_action.joint_positions)
        #     vel_action_log.append(applied_action.joint_velocities)

        # if i < 400:
        #     pos_log.append(my_husky.get_joint_positions())
        #     vel_log.append(my_husky.get_joint_velocities())
        #
        #     mode = 'vec'
        #     # if mode == 'scalar':
        #     if mode == 'vec':
        #         # joint_pos_action[0] = np.pi / 8 / (400 - i)
        #         # joint_vel_action[0] = 0.001
        #         joint_pos_action = np.zeros(13)
        #         joint_vel_action = np.zeros(13)
        #         # my_husky.set_joint_positions(joint_pos_action)
        #         # my_husky.set_joint_velocities(joint_vel_action)
        #         joint_vel_action[-4:] = 0.1
        #         my_husky.set_joint_velocities(joint_vel_action)
        #     else:
        #         pos = np.pi / 8 / (400 - i)
        #         vel = 0.001
        #         my_husky.set_joint_positions(pos, my_husky.get_dof_index(JOINT_NAMES[0]))
        #         my_husky.set_joint_velocities(vel, my_husky.get_dof_index(JOINT_NAMES[0]))
        #
        #     applied_action = my_husky.get_applied_action()
        #     pos_action_log.append(applied_action.joint_positions)
        #     vel_action_log.append(applied_action.joint_velocities)

        step_sizes = [500, 500, 500, 300, 300, 200, 150]
        accum = np.cumsum(step_sizes)
        if i < sum(step_sizes):
            pos_log.append(my_husky.get_joint_positions())
            vel_log.append(my_husky.get_joint_velocities())
            dof_log.append(my_husky.get_local_pose()[0].mean())
            applied_action = my_husky.get_applied_action()
            pos_action_log.append(applied_action.joint_positions)
            vel_action_log.append(applied_action.joint_velocities)

            # joint_pos_action[i//step_size] = -np.pi / 4 * (i % step_size) / step_size
            j = np.argmax(accum > i)
            step_size = step_sizes[j]
            joint_vel_action = np.zeros(13)
            joint_vel_action[j] = -0.12 * (-1 if j == 5 else 1)
            joint_vel_action[-4:] = 0.3
            # my_husky.set_joint_positions(joint_pos_action)
            my_husky.set_joint_velocities(joint_vel_action)

        # elif sum(step_sizes) <= i < sum(step_sizes) + 300:
        #     pos_log.append(my_husky.get_joint_positions())
        #     vel_log.append(my_husky.get_joint_velocities())
        #     dof_log.append(my_husky.get_local_pose()[0].mean())
        #     applied_action = my_husky.get_applied_action()
        #     pos_action_log.append(applied_action.joint_positions)
        #     vel_action_log.append(applied_action.joint_velocities)
        #
        #     joint_pos_action = np.zeros(13)
        #     joint_vel_action = np.zeros(13)
        #     joint_vel_action[-4:] = 0.5
        #     my_husky.set_joint_positions(joint_pos_action)
        #     my_husky.set_joint_velocities(joint_vel_action)

        # if i < 1000:
        #     pos_log.append(my_husky.get_joint_positions())
        #     vel_log.append(my_husky.get_joint_velocities())
        #
        #     before_pos = my_husky.get_joint_positions()
        #     mask = np.ones(13)
        #     mask[-4:] = 0
        #     now_pos = before_pos * (mask - i / 1000)
        #     my_husky.set_joint_positions(now_pos)
        #     my_husky.set_joint_velocities(np.zeros(13))
        #
        #     applied_action = my_husky.get_applied_action()
        #     pos_action_log.append(applied_action.joint_positions)
        #     vel_action_log.append(applied_action.joint_velocities)

        i += 1

pos_log = np.array(pos_log)
vel_log = np.array(vel_log)
pos_action_log = np.array(pos_action_log)
vel_action_log = np.array(vel_action_log)
fig, ax = plt.subplots(4, 1, figsize=(10, 20), dpi=150)
for k in range(13):
    c = plt.cm.tab20c.colors[k]
    ax[0].plot(pos_log[:, k], label=JOINT_NAMES[k], c=c)
    ax[1].plot(vel_log[:, k], label=JOINT_NAMES[k], c=c)
    ax[2].plot(pos_action_log[:, k], label=JOINT_NAMES[k], c=c)
    ax[3].plot(vel_action_log[:, k], label=JOINT_NAMES[k], c=c)

ax[2].plot(dof_log, label="dof", c='k')

ax[0].legend(loc="upper left")
ax[1].legend(loc="upper left")
ax[2].legend(loc="upper left")
ax[3].legend(loc="upper left")

ax[0].set_title("Joint Position")
ax[1].set_title("Joint Velocity")
ax[2].set_title("Joint Position Action")
ax[3].set_title("Joint Velocity Action")

ax[1].set_ylim(-1.0, 1.0)
ax[3].set_ylim(-1.0, 1.0)

# plt.savefig(f"{JOINT_NAMES[j]}_pos_vel.png")
plt.savefig(f'fr_joint1_move.png')

print("Done")

simulation_app.close()
