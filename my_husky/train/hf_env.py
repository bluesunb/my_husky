# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import gym
from gym import spaces
import numpy as np
import math
import carb

class HuskyEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        skip_frame=1,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
        max_episode_length=360,
        seed=0,
        headless=True,
    ) -> None:
        from omni.isaac.kit import SimulationApp

        self.headless = headless
        self._simulation_app = SimulationApp({"headless": self.headless, "anti_aliasing": 0})
        self._skip_frame = skip_frame
        self._dt = physics_dt * self._skip_frame
        self._max_episode_length = max_episode_length
        self._steps_after_reset = int(rendering_dt / physics_dt)

        from omni.isaac.core import World
        from husky_robot import MyRobot, JOINT_NAMES
        from omni.isaac.core.objects import VisualCuboid, VisualSphere
        from omni.isaac.core.utils.nucleus import get_assets_root_path
        # from differential_controller import DifferentialController

        self._my_world = World(physics_dt=physics_dt, rendering_dt=rendering_dt, stage_units_in_meters=1.0)
        self._my_world.scene.add_default_ground_plane()

        asset_path = "/home/bluesun/Desktop/Husky/husky_fr3/world4.usd"

        self.husky: MyRobot = self._my_world.scene.add(
            MyRobot(prim_path="/World/husky",
                    name="my_husky",
                    usd_path=asset_path,
                    position=np.array([0, 0.0, 0.1]),
                    orientation=np.array([1, -1, 0, 0]),)   # max reach: 0.725
        )
        # self.husky_controller = DifferentialController(name="simple_control", wheel_radius=0.0325, wheel_base=0.1125)

        self.goal = self._my_world.scene.add(
            VisualSphere(
                prim_path="/new_cube_1",
                name="visual_cube",
                position=np.array([0.60, 0.30, 0.5]),
                radius=0.1,
                color=np.array([1.0, 0, 0]),
            )
        )

        self.seed(seed)
        self.reward_range = (-float("inf"), float("inf"))
        # gym.Env.__init__(self)

        super().__init__()
        self.action_space = spaces.Box(low=-1, high=1.0, shape=(6,), dtype=np.float32)
        # 7: joint position, 7: joint velocity, 3: target_pos - end_effector_pos
        self.observation_space = spaces.Box(low=float("inf"), high=float("inf"), shape=(6 + 6 + 3,), dtype=np.float32)

        self.max_velocity = 0.5
        # self.max_angular_velocity = math.pi
        self.reset_counter = 0
        self.env_counter = 0

        self.joint_names = JOINT_NAMES
        self.husky_max_reach = 0.725

        return

    def get_dt(self):
        return self._dt

    def get_joint_limits(self):
        joint_limits = self.husky_controller.get_joint_limits()[self.joint_dof_idx] # aligned
        return joint_limits

    def apply_robot_action(self, action: np.ndarray, debug: bool=False):
        """
        Note that action is ALLIGNED
        and joint_limits, joint_pos are ALLIGNED also.
        """
        from omni.isaac.core.utils.types import ArticulationAction
        vel_sign = ((np.sign(action) + 1) // 2).astype(int)      # aligned
        joint_limits = np.diag(self.joint_limits[:, vel_sign]) * 0.975   # aligned

        joint_pos = self.husky.get_joint_positions()[self.joint_dof_idx]     # aligned
        # weight = np.where(joint_pos / joint_limits > 0.8, 1 - joint_pos / joint_limits, 1)  # aligned
        # action = action * weight    # aligned
        action = np.where(np.abs(joint_pos) > np.abs(joint_limits), 0, action)    # aligned

        if debug:
            print(f"action: {action}")

        robot_action = np.zeros(13)
        # for i, idx in enumerate(self.joint_dof_idx):
        #     robot_action[idx] = action[i]
        robot_action[self.joint_dof_idx] = action[:]
        robot_action = ArticulationAction(joint_velocities=robot_action)    # original
        self.husky_controller.apply_action(robot_action)

    def step(self, action: np.ndarray, debug: bool=False):
        """
        Note that action is ALLIGNED
        """
        prev_dist = np.linalg.norm(self.get_displacement())
        forward_velocity = action * self.max_velocity   # aligned

        for i in range(self._skip_frame):
            self.apply_robot_action(forward_velocity, debug=debug)
            self._my_world.step(render=False)

        observations = self.get_observations()
        info = {}
        done = False
        if self._my_world.current_time_step_index - self._steps_after_reset >= self._max_episode_length:
            done = True
            info['msg'] = "max episode length reached"

        current_dist = np.linalg.norm(observations[-3:])
        reward = prev_dist - current_dist
        if current_dist < 0.1:
            done = True
            info['msg'] = "reach goal"
        return observations, reward, done, info

    def reset(self):
        self._my_world.reset()

        self.husky_controller = self.husky.get_articulation_controller()
        self.joint_dof_idx = [self.husky.get_dof_index(name) for name in self.joint_names[:self.action_space.shape[0]]]
        self.joint_limits = self.get_joint_limits()

        self.reset_counter = 0
        self.env_counter += 1

        r = self.husky_max_reach * (np.random.rand() * 0.2 + 0.8)
        x = np.random.rand() * r
        y = np.random.rand() * np.sqrt(r ** 2 - x ** 2) * np.random.choice([-1, 1])
        z = np.sqrt(r ** 2 - x ** 2 - y ** 2)
        self.goal.set_world_pose(np.array([x, y, z]) + np.array([0.3, 0.0, 0.7]))
        observations = self.get_observations()

        return observations

    def get_displacement(self):
        import omni.usd
        left_finger = self._my_world.stage.GetPrimAtPath('/World/husky/fr3_leftfinger')
        right_finger = self._my_world.stage.GetPrimAtPath('/World/husky/fr3_rightfinger')
        # left_finger = self._my_world.stage.GetPrimAtPath('/World/husky/fr3_finger_joint1')
        # right_finger = self._my_world.stage.GetPrimAtPath('/World/husky/fr3_finger_joint2')
        left_finger_pos = omni.usd.get_world_transform_matrix(left_finger).ExtractTranslation()
        right_finger_pos = omni.usd.get_world_transform_matrix(right_finger).ExtractTranslation()

        mid_pos = (left_finger_pos + right_finger_pos) / 2
        goal_world_position, _ = self.goal.get_world_pose()
        return goal_world_position - mid_pos

    def get_observations(self):
        self._my_world.render()
        displacement = self.get_displacement()
        joint_pos = self.husky.get_joint_positions()[self.joint_dof_idx]
        joint_vel = self.husky.get_joint_velocities()[self.joint_dof_idx]
        return np.concatenate([joint_pos,   # aligned
                               joint_vel,   # aligned
                               displacement])

    def render(self, mode="human"):
        return

    def close(self):
        self._simulation_app.close()
        return

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        np.random.seed(seed)
        return [seed]
