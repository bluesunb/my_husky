import numpy as np

from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers import BaseController
from husky_fr3_robot import HuskyFR3Robot

from typing import Optional
import copy


class AggregationController:
    def __init__(self, robot_articulation: HuskyFR3Robot):
        self.robot = robot_articulation
        self.has_reset = {'joint_positions': True,
                          'joint_velocities': True,
                          'joint_efforts': True}

    def forward(self,
                joint_action: Optional[ArticulationAction] = None,
                wheel_action: Optional[ArticulationAction] = None, ):

        if joint_action is None:
            joint_action = ArticulationAction()
        if wheel_action is None:
            wheel_action = ArticulationAction()

        applied_action = self.robot.get_applied_action()
        action = ArticulationAction(joint_indices=self.robot.joint_dof_indices)
        for attr in ["joint_positions", "joint_velocities", "joint_efforts"]:
            joint_vec = copy.deepcopy(getattr(joint_action, attr, None))
            wheel_vec = copy.deepcopy(getattr(wheel_action, attr, None))
            aggregated_vec = copy.deepcopy(getattr(applied_action, attr, None))

            if not (joint_vec is None and wheel_vec is None):
                if self.has_reset[attr]:
                    self.has_reset[attr] = False
                else:
                    aggregated_vec = aggregated_vec[self.robot.joint_dof_indices]
                    # if joint_vec is not None and joint_vec[0] is not None:
                    #     if not isinstance(joint_vec, np.ndarray):
                    #         joint_vec = np.array(joint_vec)
                    #     tmp = self.robot.joint_dof_indices[:len(joint_vec)]
                    #     rank = np.empty_like(tmp)
                    #     rank[np.argsort(tmp)] = np.arange(len(tmp))
                    #     joint_vec = joint_vec[rank]

                if joint_vec is None:
                    aggregated_vec[-len(wheel_vec):] = wheel_vec
                elif wheel_vec is None:
                    aggregated_vec[:len(joint_vec)] = joint_vec
                else:
                    aggregated_vec[:len(joint_vec)] = joint_vec
                    aggregated_vec[-len(wheel_vec):] = wheel_vec
                setattr(action, attr, aggregated_vec)

        return action

    def reset(self):
        self.has_reset = {'joint_positions': True,
                          'joint_velocities': True,
                          'joint_efforts': True}
