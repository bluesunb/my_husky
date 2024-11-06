#
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, unknown = parser.parse_known_args()


from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.objects import VisualCuboid
# from omni.isaac.wheeled_robots.robots import WheeledRobot
from husky_fr3_robot import HuskyFR3Robot
# from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
# from stable_baselines_example.differential_controller import DifferentialController
from husky_fr3_controller import HuskyFR3Controller
import numpy as np
import carb
from omni.isaac.core.utils.types import ArticulationAction

my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
husky_fr3_asset_path = "/home/bluesun/Desktop/Husky/husky_fr3/world4.usd"
# husky_fr3_asset_path = "/home/bluesun/Desktop/Husky/world.usd"
my_husky_fr3: HuskyFR3Robot = my_world.scene.add(
    HuskyFR3Robot(
        prim_path="/World/husky",
        # prim_path="/husky_fr3",
        name="my_robot",
        create_robot=True,
        usd_path=husky_fr3_asset_path,
        # position=np.array([0, 0.0, 0.5]),
        # orientation=np.array([0, 1, 0, 0]),
    )
)

# cube = my_world.scene.add(
#     VisualCuboid(prim_path="/World/Cube",
#                  name="my_cube",
#                  position=np.array([0.6, 0.3, 0.05]),
#                  size=0.1,
#                  color=np.array([1.0, 0.0, 0.0,]))
# )
my_world.scene.add_default_ground_plane()
my_controller = HuskyFR3Controller(name="simple_control",
                                   gripper=my_husky_fr3.gripper,
                                   robot_articulation=my_husky_fr3,
                                   wheel_radius=0.3,
                                   wheel_base=0.1125)
my_world.reset()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    # print('=====================================')
    # print(f'pos: {my_husky_fr3.get_joint_positions()[my_husky_fr3.joint_dof_indices]}')
    # print(f'vel: {my_husky_fr3.get_joint_velocities()[my_husky_fr3.joint_dof_indices]}')
    # print('=====================================')
    # if i < 3 or i % 1000 == 0:
    #     print(f'====================={i}=====================')
    #     print(f'robot pos: {my_husky_fr3.get_local_pose()}')
    #     print(f'robot vel: {my_husky_fr3.get_linear_velocity()}')
    #     print(f'robot ang: {my_husky_fr3.get_angular_velocity()}')
    #     print(f'robot joint pos: {my_husky_fr3.get_joint_positions()}')
    #     print(f'robot joint vel: {my_husky_fr3.get_joint_velocities()}')
    #     print(f'robot action: {my_husky_fr3.get_applied_action()}')
    #     print(f'robot gripper pos: {my_husky_fr3.gripper.get_joint_positions()}')

    # if my_world.is_playing():
    #     if my_world.current_time_step_index == 0:
            # my_world.scene.remove_object('my_robot')
            # robot_config = dict(prim_path="/World/husky_fr3",
            #                     name="my_robot",
            #                     create_robot=True,
            #                     usd_path=husky_fr3_asset_path,
            #                     # position=np.array([0, 0.0, 0.0]),
            #                     # translation = np.array([0, 0, 0.5])
            #                     orientation = np.array([1, 0, 0, 0])
            #                     )
            # my_husky_fr3: HuskyFR3Robot = my_world.scene.add(HuskyFR3Robot(**robot_config))
            # my_world.reset()
            # my_controller.reset()
        # if i == 1000:
        #     my_husky_fr3.set_robot_joint_positions([-0.5, 0.5, -0.5, 0.5, 0.5, 0.5, 0.5, -2.6953744963975623e-05, -2.692015186767094e-05, 0.0, 0.0, 0.0, 0.0])

    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
            # pass
        # if 0 <= i < 1000:
        #     # forward
        #     my_husky_fr3.apply_action(my_controller.wheel_forward(command=np.array([0.2, 0])))
        #     # print('forward: ' + str([f'{v:.4e}' for v in my_husky_fr3.get_linear_velocity()]))
        #     # print(f'vel: {my_husky_fr3.get_joint_velocities()}')
        #     # print(f'pos: {my_husky_fr3.get_joint_positions()}')
        #     # print(f'act: {my_husky_fr3.get_applied_action()}')
        #
        # elif 1000 <= i < 1300:
        #     # rotate
        #     my_husky_fr3.apply_action(my_controller.wheel_forward(command=np.array([0.0, np.pi])))
        #     print('rotate: ' + str([f'{v:.4e}' for v in my_husky_fr3.get_angular_velocity()]))
        # elif i == 1300:
        #     joint_target_positions = {'fr3_joint1': np.degrees(0.007),
        #                               'fr3_joint2': np.degrees(-0.37),
        #                               'fr3_joint3': np.degrees(0),
        #                               'fr3_joint4': np.degrees(-1.81),
        #                               'fr3_joint5': np.degrees(0),
        #                               'fr3_joint6': np.degrees(1.537),
        #                               'fr3_joint7': np.degrees(0.241),
        #                               'fr3_finger_joint1': 4,
        #                               'fr3_finger_joint2': 4}
        #     joint_target_positions_vec = np.array([val for val in joint_target_positions.values()])
        #     init_positions = my_husky_fr3.get_joint_positions()[my_husky_fr3.joint_dof_indices]
        #     delta = (joint_target_positions_vec - init_positions[:len(joint_target_positions_vec)]) / 2000
        #     delta = np.pad(delta, (0, 4), 'constant')
        #     for j in range(1000):
        #         if my_world.current_time_step_index == 0:
        #             my_world.reset()
        #             my_controller.reset()
        #         action_pos = my_husky_fr3.get_joint_positions()[my_husky_fr3.joint_dof_indices] + delta
        #         action_pos = np.pad(action_pos[:len(joint_target_positions_vec)], (0, 4), 'constant', constant_values=0)
        #         my_husky_fr3.apply_action(ArticulationAction(joint_positions=action_pos))
        #         my_world.step(render=True)
        #
        # elif 1300 < i < 2000:
        #     # forward
        #     my_husky_fr3.apply_action(my_controller.wheel_forward(command=np.array([0.05, 0])))
        #     # print(f'end forward: {my_husky_fr3.get_applied_action()}')
        # elif i == 2000:
        #     i = 0
        # i += 1
    if args.test is True:
        break

simulation_app.close()
