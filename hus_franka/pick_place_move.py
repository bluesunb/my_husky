from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pick_place_task import HuskyFR3Task
# from omni.isaac.franka.controllers import PickPlaceController
from husky_fr3_controller import HuskyFR3Controller
from omni.isaac.core import World
import numpy as np

my_world = World(stage_units_in_meters=1.0)
my_task = HuskyFR3Task("my_task")
my_world.add_task(my_task)
my_world.reset()
task_params = my_task.get_params()
my_robot = my_world.scene.get_object(task_params["robot_name"]["value"])
my_controller = HuskyFR3Controller(name="pick_place_controller",
                                   gripper=my_robot.gripper,
                                   robot_articulation=my_robot,
                                   wheel_radius=0.3,
                                   wheel_base=0.1125)
articulation_controller = my_robot.get_articulation_controller()

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
        observations = my_world.get_observations()
        actions = my_controller.pick_place_forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            end_effector_offset=np.array([0, 0.005, -0.015]),
        )
        if my_controller.is_done():
            print("done picking and placing")
        articulation_controller.apply_action(actions)
simulation_app.close()
