from utils.path_registry import PathRegistry

from utils.json_manager import JsonManager

from utils.robot_pose import RobotPose


class RobotPoseRegistry:
    """
    This class will read the json file and set poses as its attributes
    """
    drop: RobotPose = None
    home_pose: RobotPose = None
    drop_pose: RobotPose = None
    wall3_q5_t6_after_top_recoil_pose: RobotPose = None
    wall3_q5_t5: RobotPose = None
    wall3_q5_t6: RobotPose = None

    @classmethod
    def form_poses(cls, json_pose_path: str = PathRegistry.poses_json_fullpath):
        jm = JsonManager(json_pose_path)
        json_poses = jm.read_json()
        for pose_name in json_poses.keys():
            pose = RobotPose(pose_name, json_poses[pose_name])
            setattr(cls, pose_name, pose)
