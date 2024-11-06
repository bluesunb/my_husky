import omni
import omni.kit.app
import omni.kit.viewport
import omni.kit.commands
import omni.kit.selection


def create_colored_cube():
    # Initialize the Omniverse Kit application
    app = omni.kit.app.get_app_interface()

    # Create a cube entity
    cube_entity = omni.kit.commands.execute("CreateEntity", type="cube")

    # Define colors for each side
    side_colors = [
        (1, 0, 0),  # Red
        (0, 1, 0),  # Green
        (0, 0, 1),  # Blue
        (1, 1, 0),  # Yellow
        (0, 1, 1),  # Cyan
        (1, 0, 1),  # Magenta
    ]

    # Iterate through the cube's sides and assign materials with colors
    for side_index, color in enumerate(side_colors):
        material_name = f"Material_{side_index}"
        omni.kit.commands.execute("CreateMaterial", name=material_name)
        omni.kit.commands.execute("SetMaterialColor", name=material_name, color=color)

        # Assign the material to the corresponding side of the cube
        omni.kit.commands.execute(
            "SetMaterialForGeomSubset",
            geom=cube_entity,
            subset_index=side_index,
            material=material_name,
        )

    # Place the cube in the scene
    omni.kit.commands.execute("SetTransform", entity=cube_entity, position=(0, 0, 0))


if __name__ == "__main__":
    create_colored_cube()
