from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})
from omni.isaac.core.utils.extensions import enable_extension

enable_extension("omni.isaac.ros_bridge")

from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core import World
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core_nodes.impl.extension import omni
import numpy as np
import carb
import os


class Newworld(object):

    def __init__(self):
        self.count = 1
        self.husky_asset_path = os.path.abspath('/home/bluesun/Desktop/Husky/world.usd')

    def setup_scene(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_ground_plane()
        self.world.scene.add(
            WheeledRobot(
                prim_path="/World/husky",
                wheel_dof_names=['front_left_wheel',
                                 'front_right_wheel',
                                 'rear_left_wheel',
                                 'rear_right_wheel'],
                wheel_dof_indices=[0, 1, 2, 3],
                create_robot=True,
                name="Husky",
                usd_path=self.husky_asset_path,
                position=np.array([0, 0.0, 0.0])))


        self.keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/husky/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS1SubscribeTwist", "omni.isaac.ros_bridge.ROS1SubscribeTwist"),
                    ("ScaleToFromStageUnits", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                    ("Break3Vector1", "omni.graph.nodes.BreakVector3"),
                    ("Break3Vector2", "omni.graph.nodes.BreakVector3"),
                    ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
                    ("ConstantToken1", "omni.graph.nodes.ConstantToken"),
                    ("ConstantToken2", "omni.graph.nodes.ConstantToken"),
                    ("ConstantToken3", "omni.graph.nodes.ConstantToken"),
                    ("ConstantToken4", "omni.graph.nodes.ConstantToken"),
                    ("ArrayIndex1", "omni.graph.nodes.ArrayIndex"),
                    ("ArrayIndex2", "omni.graph.nodes.ArrayIndex"),
                    ("MakeArray1", "omni.graph.nodes.MakeArray"),
                    ("MakeArray2", "omni.graph.nodes.MakeArray"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController")
                ],

                og.Controller.Keys.SET_VALUES: [
                    ("DifferentialController.inputs:maxLinearSpeed", 3.0),
                    ("DifferentialController.inputs:wheelDistance", 0.1),
                    ("DifferentialController.inputs:wheelRadius", 0.16),
                    ("ConstantToken1.inputs:value", "front_left_wheel"),
                    ("ConstantToken2.inputs:value", "front_right_wheel"),
                    ("ConstantToken3.inputs:value", "rear_left_wheel"),
                    ("ConstantToken4.inputs:value", "rear_right_wheel"),
                    ("ArrayIndex1.inputs:index", 0),
                    ("ArrayIndex2.inputs:index", 1),
                    ("MakeArray1.inputs:arraySize", 4),
                    ("MakeArray2.inputs:arraySize", 4),
                    ("ArticulationController.inputs:robotPath", "/World/husky"),
                    ("ArticulationController.inputs:usePath", True)
                ],

                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS1SubscribeTwist.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("ROS1SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),
                    ("ROS1SubscribeTwist.outputs:angularVelocity", "Break3Vector1.inputs:tuple"),
                    ("ROS1SubscribeTwist.outputs:linearVelocity", "ScaleToFromStageUnits.inputs:value"),
                    ("ScaleToFromStageUnits.outputs:result", "Break3Vector2.inputs:tuple"),
                    ("Break3Vector1.outputs:z", "DifferentialController.inputs:angularVelocity"),
                    ("Break3Vector2.outputs:x", "DifferentialController.inputs:linearVelocity"),
                    ("DifferentialController.outputs:velocityCommand", "ArrayIndex1.inputs:array"),
                    ("DifferentialController.outputs:velocityCommand", "ArrayIndex2.inputs:array"),
                    ("ArrayIndex1.outputs:value", "MakeArray2.inputs:b"),
                    ("ArrayIndex1.outputs:value", "MakeArray2.inputs:d"),
                    ("ArrayIndex2.outputs:value", "MakeArray2.inputs:a"),
                    ("ArrayIndex2.outputs:value", "MakeArray2.inputs:c"),
                    ("MakeArray2.outputs:array", "ArticulationController.inputs:velocityCommand"),
                    ("ConstantToken1.inputs:value", "MakeArray1.inputs:b"),
                    ("ConstantToken2.inputs:value", "MakeArray1.inputs:a"),
                    ("ConstantToken3.inputs:value", "MakeArray1.inputs:d"),
                    ("ConstantToken4.inputs:value", "MakeArray1.inputs:c"),
                    ("MakeArray1.outputs:array", "ArticulationController.inputs:jointNames")
                ],
            },
        )

        self.world.reset()

    def run(self):
        self.setup_scene()

        while simulation_app.is_running():
            self.world.step(render=True)

        simulation_app.close()


if __name__ == '__main__':
    new_wld = Newworld()
    new_wld.run()