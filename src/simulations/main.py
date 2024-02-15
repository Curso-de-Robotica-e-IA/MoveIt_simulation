from scene_builder import Scene
from set_up import SetUp
from utils.json_manager import JsonManager

if __name__ == "__main__":
    setup = SetUp()
    scene = Scene(setup)
    jm = JsonManager()

    # scene.add_mesh()
    # scene.add_table()
