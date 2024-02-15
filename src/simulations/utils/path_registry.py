from dataclasses import dataclass
from pathlib import Path


@dataclass
class PathRegistry:
    """
    Files paths are saved in this class.

    This class attributes should only be paths represented as strings

    Builds the fullpath in the way that MoveIt expects.

    To register a path, it should be relative to the main catkin_workspace folder.
    Add it to this class attributes as a string.

    Use method build_moveit_path to get the file fullpath if necessary.
    """
    shelf_only_3s: str = 'src/testes_estantes/scripts/teste3oandarmodelo3d/montagemPlanejamento-3andares.stl'
    shelf_4s: str = 'src/MoveIt_simulation/src/simulations/models3D/montagemPlanejamento-Completa-4andares-wMesa-modif.stl'
    support: str = 'src/MoveIt_simulation/src/simulations/models3D/montagemD405-modif.stl'
    poses_json = 'src/MoveIt_simulation/src/simulations/utils/shelf_pose.json'
    poses_json_fullpath = '/home/rcmm/Documents/kinova_ros/catkin_workspace/src/MoveIt_simulation/src/simulations/utils/shelf_pose.json'

    @staticmethod
    def build_moveit_path(relative_path: str) -> Path:
        return Path.cwd() / relative_path
