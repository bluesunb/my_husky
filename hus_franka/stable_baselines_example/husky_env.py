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
        max_episode_length=256,
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
        from omni.isaac.wheeled_robots.robots import WheeledRobot
        from omni.isaac.core.objects import VisualCuboid
        from omni.isaac.core.utils.nucleus import get_assets_root_path
        from differential_controller import DifferentialController

        self._my_world = World(physics_dt=physics_dt, rendering_dt=rendering_dt, stage_units_in_meters=1.0)
        self._my_world.scene.add_default_ground_plane()

        husky_asset_path = "/home/bluesun/Desktop/Husky/world.usd"

        self.husky: WheeledRobot = self._my_world.scene.add(
            WheeledRobot(
                prim_path="/husky",
                name="my_husky",
                wheel_dof_names=["front_left_wheel",
                                 "front_right_wheel",
                                 "rear_left_wheel",
                                 "rear_right_wheel"],
                wheel_dof_indices=[0, 1, 2, 3],
                create_robot=True,
                usd_path=husky_asset_path,
                position=np.array([0, 0.0, 0.03]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
        )
        self.husky_controller = DifferentialController(name="simple_control", wheel_radius=0.0325, wheel_base=0.1125)

        self.goal = self._my_world.scene.add(
            VisualCuboid(
                prim_path="/new_cube_1",
                name="visual_cube",
                position=np.array([0.60, 0.30, 0.05]),
                size=0.1,
                color=np.array([1.0, 0, 0]),
            )
        )

        self.seed(seed)
        self.reward_range = (-float("inf"), float("inf"))
        # gym.Env.__init__(self)

        super().__init__()
        self.action_space = spaces.Box(low=-1, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=float("inf"), high=float("inf"), shape=(16,), dtype=np.float32)

        self.max_velocity = 1
        self.max_angular_velocity = math.pi
        self.reset_counter = 0
        return

    def get_dt(self):
        return self._dt

    def step(self, action: np.ndarray):
        previous_husky_position, _ = self.husky.get_world_pose()
        # action forward velocity , angular velocity on [-1, 1]
        raw_forward = action[0]
        raw_angular = action[1]

        # we want to force the jetbot to always drive forward
        # so we transform to [0,1].  we also scale by our max velocity
        forward = (raw_forward + 1.0) / 2.0
        forward_velocity = forward * self.max_velocity

        # we scale the angular, but leave it on [-1,1] so the
        # jetbot can remain an ambiturner.
        angular_velocity = raw_angular * self.max_angular_velocity

        # we apply our actions to the jetbot
        for i in range(self._skip_frame):
            self.husky.apply_wheel_actions(
                self.husky_controller.forward(command=[forward_velocity, angular_velocity])
            )
            self._my_world.step(render=False)

        observations = self.get_observations()
        info = {}
        done = False
        if self._my_world.current_time_step_index - self._steps_after_reset >= self._max_episode_length:
            done = True
        goal_world_position, _ = self.goal.get_world_pose()
        current_husky_position, _ = self.husky.get_world_pose()
        previous_dist_to_goal = np.linalg.norm(goal_world_position - previous_husky_position)
        current_dist_to_goal = np.linalg.norm(goal_world_position - current_husky_position)
        reward = previous_dist_to_goal - current_dist_to_goal
        if current_dist_to_goal < 0.1:
            done = True
        return observations, reward, done, info

    def reset(self):
        self._my_world.reset()
        self.reset_counter = 0
        # randomize goal location in circle around robot
        alpha = 2 * math.pi * np.random.rand()
        r = 1.00 * math.sqrt(np.random.rand()) + 0.20
        self.goal.set_world_pose(np.array([math.sin(alpha) * r, math.cos(alpha) * r, 0.05]))
        observations = self.get_observations()
        return observations

    def get_observations(self):
        self._my_world.render()
        husky_world_position, husky_world_orientation = self.husky.get_world_pose()
        husky_linear_velocity = self.husky.get_linear_velocity()
        husky_angular_velocity = self.husky.get_angular_velocity()
        goal_world_position, _ = self.goal.get_world_pose()
        return np.concatenate(
            [
                husky_world_position,
                husky_world_orientation,
                husky_linear_velocity,
                husky_angular_velocity,
                goal_world_position,
            ]
        )

    def render(self, mode="human"):
        return

    def close(self):
        self._simulation_app.close()
        return

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        np.random.seed(seed)
        return [seed]
