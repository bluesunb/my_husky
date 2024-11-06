from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units, print_stage_prim_paths
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from husky_fr3_robot import HuskyFR3Robot
from husky_fr3_controller import AggregationController
from stable_baselines_example.differential_controller import DifferentialController
import numpy as np
import carb

from typing import Union
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.types import ArticulationAction

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
for _ in range(30):
    action = ArticulationAction(joint_positions=np.full(13, None),
                                joint_velocities=np.full(13, None))
    my_husky.apply_action(action, indices=None)
    my_world.step(render=True)

print(my_husky.get_applied_action().__dict__)
while simulation_app.is_running():
    my_world.step(render=True)
    # print(my_husky.get_local_pose())
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            husky_controller.reset()
            wheel_controller.reset()
            franka_controller.reset()
        if 0 <= i < 12:
            print(i)
        elif 12 <= i < 500:
            wheel_action = wheel_controller.forward(command=np.array([0.05, 0]))
            action = husky_controller.forward(wheel_action=wheel_action)
            my_husky.apply_action(action, indices=None)
        elif 500 <= i < 1000:
            wheel_action = wheel_controller.forward(command=np.array([0.0, np.pi]))
            action = husky_controller.forward(wheel_action=wheel_action)
            my_husky.apply_action(action, indices=None)
        elif i == 1000:
            pos_vel_log = np.load("pos_vel_log.npz")
            pos_log = pos_vel_log["pos_log"]
            vel_log = pos_vel_log["vel_log"]

            for pos, vel in zip(pos_log, vel_log):
                if my_world.current_time_step_index == 0:
                    i = 0
                    my_world.reset()
                    husky_controller.reset()
                    wheel_controller.reset()
                    franka_controller.reset()
                    break
                picking_position = my_cube.get_local_pose()[0] + np.array([0.0, 0.0, 0.3])
                # placing_position = calc_target_pos()
                # placing_position =np.array([-0.3, -0.3, 0.5]) / get_stage_units()
                # current_joint_positions = my_husky.get_joint_positions()[my_husky.joint_dof_indices][:7]
                # end_effector_offset = np.array([0, 0.005, -0.015])
                # joint_action = franka_controller.forward(picking_position=picking_position,
                #                                          placing_position=placing_position,
                #                                          current_joint_positions=current_joint_positions,
                #                                          end_effector_offset=end_effector_offset)
                joint_action = ArticulationAction(joint_positions=pos, joint_velocities=vel)
                wheel_action = wheel_controller.forward(command=np.array([0.0, 0.0]))
                action = husky_controller.forward(joint_action=joint_action, wheel_action=wheel_action)
                # print(f'action pos:\t{", ".join(f"{(x or 0):.4e}" for x in action.joint_positions)}')
                # print(f'action vel:\t{", ".join(f"{(x or 0):.4e}" for x in action.joint_velocities)}')
                # print(f'robot pos:\t{", ".join(f"{(x or 0):.4e}" for x in my_husky.get_joint_positions()[my_husky.joint_dof_indices])}')
                # print(f'robot vel:\t{", ".join(f"{(x or 0):.4e}" for x in my_husky.get_joint_velocities()[my_husky.joint_dof_indices])}')
                my_husky.apply_action(action, indices=None)
                my_world.step(render=True)
        else:
            i = 0
        i += 1

simulation_app.close()
