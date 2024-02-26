import json
from json import JSONDecodeError
from pathlib import Path

from utils.robot_pose import RobotPose

from utils.path_registry import PathRegistry


class JsonManager:

    def __init__(self, file: str = ''):
        self.file = file

    def read_json(self, file: str = '') -> dict:
        file = self.assure_file_not_empty(file)
        file = Path(file)

        f = open(self.file or file, 'r')

        try:
            json_file = json.load(f)
            f.close()
        except (JSONDecodeError,):
            print("Json is empty. Returning an empty dict")
            return {}

        return json_file

    def write_json(self, data: dict, file: str = '', mode: str = 'w'):
        file = self.assure_file_not_empty(file)
        file = Path(file)

        f = open(self.file or file, mode)

        json.dump(data, f, indent=4)

    def append_pose_obj_list_to_json(self, robot_pose: RobotPose, file: str = PathRegistry.poses_json_fullpath):
        """
        Saves a RobotPose object into a json registry.
        """
        robot_pose = {str(robot_pose): robot_pose.pose_list}
        self.append_to_json(robot_pose, file=file)

    def append_list_to_json(self, pose_list: list, name: str = '', file: str = PathRegistry.poses_json_fullpath):
        """
        Saves a list with a pose object into a json registry.
        """
        robot_pose = {name: pose_list}
        self.append_to_json(robot_pose, file=file)

    def append_to_json(self, data: dict, file: str = '', mode: str = 'w'):
        file = self.assure_file_not_empty(file)
        file = Path(file)

        json_data = self.read_json(str(file))
        json_data.update(data)

        with open(self.file or file, mode) as f:
            json.dump(json_data, f, indent=4)

    def assure_file_not_empty(self, file: str) -> str:
        """
        Guarantees that the file attribute and file argument were passed.
        If they weren't, raises FilePathEmpty exception.
        """
        if self.file:
            return self.file

        if file:
            return file

        raise FilePathEmpty


class FilePathEmpty(Exception):

    def __init__(self):
        super().__init__()


if __name__ == '__main__':
    jm = JsonManager()
    jm.read_json()
