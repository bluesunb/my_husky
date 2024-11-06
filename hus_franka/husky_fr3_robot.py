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
                 name: str = "husky_franka",
                 usd_path: Optional[str] = None,
                 create_robot: Optional[bool] = False,
                 position: Optional[np.ndarray] = None,
                 orientation: Optional[np.ndarray] = None,
                 **kwargs) -> None:

        prim = get_prim_at_path(prim_path)
        self._end_effector = None
        self._gripper = None
        if not prim.IsValid():
            if not prim.IsValid():
                if create_robot:
                    usd_path = "/home/bluesun/Desktop/Husky/husky_fr3/world3.usd"
                    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
                else:
                    carb.log_error("no prim at path %s", prim_path)
        self._end_effector_prim_path = prim_path + "/fr3_hand"
        gripper_dof_names = [name for name in JOINT_NAMES if 'finger' in name]
        gripper_open_positions = np.array([0.05, 0.05]) / get_stage_units()
        gripper_closed_positions = np.array([0.0, 0.0])

        super().__init__(
            prim_path=prim_path,
            name=name,
            position=position,
            orientation=orientation,
            articulation_controller=None,
            **kwargs
        )

        if gripper_dof_names is not None:
            deltas = np.array([0.05, 0.05]) / get_stage_units()
            self._gripper = ParallelGripper(
                end_effector_prim_path=self._end_effector_prim_path,
                joint_prim_names=gripper_dof_names,
                joint_opened_positions=gripper_open_positions,
                joint_closed_positions=gripper_closed_positions,
                action_deltas=deltas,
            )

        self._robot_dof_names = JOINT_NAMES.copy()
        self._robot_dof_indices = list(range(len(self._robot_dof_names)))
        self._articulation_view.get_dof_types()

    @property
    def joint_dof_indices(self):
        return self._robot_dof_indices

    def get_dof_index(self, dof_name: str) -> int:
        if "panda" in dof_name:
            dof_name = JOINT_MAP[dof_name]
        return super().get_dof_index(dof_name)

    def get_robot_joint_positions(self):
        full_dofs_positions = self.get_joint_positions()
        joint_positions = [full_dofs_positions[i] for i in self._robot_dof_indices]
        return joint_positions

    def set_robot_joint_positions(self, positions):
        full_dofs_positions = [None] * self.num_dof
        for i in range(self._num_robot_dof):
            full_dofs_positions[self._robot_dof_indices[i]] = positions[i]
        self.set_joint_positions(positions=np.array(full_dofs_positions))
        return

    def get_robot_joint_velocities(self):
        full_dofs_velocities = self.get_joint_velocities()
        joint_velocities = [full_dofs_velocities[i] for i in self._robot_dof_indices]
        return joint_velocities

    def set_robot_joint_velocities(self, velocities):
        full_dofs_velocities = [None] * self.num_dof
        for i in range(self._num_robot_dof):
            full_dofs_velocities[self._robot_dof_indices[i]] = velocities[i]
        self.set_joint_velocities(velocities=np.array(full_dofs_velocities))
        return

    @property
    def end_effector(self):
        return self._end_effector

    @property
    def gripper(self) -> ParallelGripper:
        return self._gripper

    def apply_action(self, control_actions: ArticulationAction, indices=None) -> None:
        # indices = self.joint_dof_indices.copy()
        # control_actions.joint_indices = indices
        if indices is None:
            super().apply_action(control_actions)
        elif indices == 'robot':
            indices = np.argsort(self._robot_dof_indices)
            if control_actions.joint_positions is not None:
                control_actions.joint_positions = control_actions.joint_positions[indices]
            if control_actions.joint_velocities is not None:
                control_actions.joint_velocities = control_actions.joint_velocities[indices]
            if control_actions.joint_efforts is not None:
                control_actions.joint_efforts = control_actions.joint_efforts[indices]
            super().apply_action(control_actions)

    def apply_robot_joint_actions(self, actions: ArticulationAction):
        actions_length = actions.get_length()
        if actions_length is not None and actions_length != self._num_robot_dof:
            raise ValueError("Actions length %s does not match robot dof %s" % (actions_length, self._num_robot_dof))
        joint_actions = ArticulationAction()
        if actions.joint_positions is not None:
            joint_actions.joint_positions = np.zeros(self.num_dof)
            for i in range(self._num_robot_dof):
                joint_actions.joint_positions[self._robot_dof_indices[i]] = actions.joint_positions[i]
        if actions.joint_velocities is not None:
            joint_actions.joint_velocities = np.zeros(self.num_dof)
            for i in range(self._num_robot_dof):
                joint_actions.joint_velocities[self._robot_dof_indices[i]] = actions.joint_velocities[i]
        if actions.joint_efforts is not None:
            joint_actions.joint_efforts = np.zeros(self.num_dof)
            for i in range(self._num_robot_dof):
                joint_actions.joint_efforts[self._robot_dof_indices[i]] = actions.joint_efforts[i]
        self.apply_action(control_actions=joint_actions)
        return

    def initialize(self, physics_sim_view=None):
        super().initialize(physics_sim_view)
        if self._robot_dof_names is not None:
            self._robot_dof_indices = [
                self.get_dof_index(self._robot_dof_names[i]) for i in range(len(self._robot_dof_names))
            ]
        elif self._robot_dof_indices is None:
            carb.log_error("No joint dof indices or names defined for robot %s" % self.name)

        self._num_robot_dof = len(self._robot_dof_indices)

        self._end_effector = RigidPrim(prim_path=self._end_effector_prim_path,
                                       name=self.name + "_end_effector")
        self._end_effector.initialize(physics_sim_view)
        self._gripper.initialize(
            physics_sim_view=physics_sim_view,
            articulation_apply_action_func=self.apply_action,
            get_joint_positions_func=self.get_joint_positions,
            set_joint_positions_func=self.set_joint_positions,
            dof_names=self._robot_dof_names,
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
        return self._robot_dof_names, self._robot_dof_indices