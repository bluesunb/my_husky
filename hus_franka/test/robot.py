from typing import Optional
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import get_prim_at_path, define_prim

from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units, print_stage_prim_paths

import carb
from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper

from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.franka import Franka
from omni.isaac.wheeled_robots.robots import WheeledRobot

PANDA_JOINT_NAMES = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
    "fr3_finger_joint1",
    "fr3_finger_joint2",
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
]

JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
    "fr3_finger_joint1",
    "fr3_finger_joint2",
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel"
]

JOINT_MAP = {k: v for k, v in zip(PANDA_JOINT_NAMES, JOINT_NAMES)}

class HuskyFR3Robot(Robot):
    def __init__(self,
                 prim_path: str,
                 name: str = "my_husky",
                 usd_path: Optional[str] = None,
                 create_robot: Optional[bool] = False,
                 position: Optional[np.ndarray] = None,
                 orientation: Optional[np.ndarray] = None):

        prim = get_prim_at_path(prim_path)
        self._end_effector = None
        self._gripper = None
        if not prim.IsValid():
            if create_robot:
                usd_path = "/home/bluesun/Desktop/Husky/husky_fr3/world4.usd"
                add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            else:
                raise ValueError("Invalid prim path")
        super().__init__(prim_path=prim_path,
                         name=name,
                         position=position,
                         orientation=orientation,
                         articulation_controller=None)

        self._end_effector_prim_path = prim_path + "/fr3_hand"
        gripper_dof_names = [name for name in JOINT_NAMES if "finger" in name]
        gripper_open_positions = np.array([0.05, 0.05]) / get_stage_units()
        gripper_closed_positions = np.array([0.0, 0.0])

        if gripper_dof_names is not None:
            deltas = np.array([0.05, 0.05]) / get_stage_units()
            self._gripper = ParallelGripper(
                end_effector_prim_path=self._end_effector_prim_path,
                joint_prim_names=gripper_dof_names,
                joint_opened_positions=gripper_open_positions,
                joint_closed_positions=gripper_closed_positions,
                action_deltas=deltas
            )

    @property
    def joint_dof_indices(self):
        return [self.get_dof_index(name) for name in self.dof_names]

    @property
    def absolute_joint_dof_indices(self):
        return [self.get_dof_index(name) for name in JOINT_NAMES]

    def get_dof_index(self, dof_name: str) -> int:
        return super().get_dof_index(JOINT_MAP.get(dof_name, dof_name))

    def get_joint_positions(self, joint_indices=None):
        if joint_indices is not None:
            return super().get_joint_positions(joint_indices)
        return super().get_joint_positions(self.absolute_joint_dof_indices)

    def set_joint_positions(self, positions, joint_indices=None):
        if joint_indices is not None:
            super().set_joint_positions(positions, joint_indices)
        else:
            # feed unordered action
            super().set_joint_positions(positions, self.absolute_joint_dof_indices)

    def get_joint_velocities(self, joint_indices=None) -> np.ndarray:
        if joint_indices is not None:
            return super().get_joint_velocities(joint_indices)
        return super().get_joint_velocities(self.absolute_joint_dof_indices)

    def set_joint_velocities(self, velocities, joint_indices=None) -> None:
        if joint_indices is not None:
            super().set_joint_velocities(velocities, joint_indices)
        else:
            super().set_joint_velocities(velocities, self.absolute_joint_dof_indices)

    @property
    def end_effector(self):
        return self._end_effector

    @property
    def gripper(self) -> ParallelGripper:
        return self._gripper

    def initialize(self, physics_sim_view=None):
        super().initialize(physics_sim_view)
        self._end_effector = RigidPrim(prim_path=self._end_effector_prim_path,
                                       name=self.name + "_end_effector")
        self._end_effector.initialize(physics_sim_view)
        self._gripper.initialize(
            physics_sim_view=physics_sim_view,
            articulation_apply_action_func=self.apply_action,
            get_joint_positions_func=self.get_joint_positions,
            set_joint_positions_func=self.set_joint_positions,
            dof_names=self.dof_names,
        )

        return

    def post_reset(self) -> None:
        super().post_reset()
        self._gripper.post_reset()
        for i, joint_name in enumerate(JOINT_NAMES):
            self._articulation_controller.switch_dof_control_mode(
                dof_index=self.get_dof_index(joint_name),
                mode="velocity" if 'wheel' in joint_name else "position"
            )
        return

    def get_articulation_controller_properties(self):
        return self.dof_names, self.joint_dof_indices