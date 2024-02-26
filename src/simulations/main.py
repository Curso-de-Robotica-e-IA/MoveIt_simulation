from pathlib import Path
path = Path.cwd()
print(f"{path = }")
from classes.scene_builder import Scene
from classes.set_up import SetUp
from utils.path_registry import PathRegistry
from utils.robot_pose_registry import RobotPoseRegistry
from utils.json_manager import JsonManager

def run_simulation_w3_q5_t6_after_top_recoil_pose():
    ...

if __name__ == "__main__":
    setup = SetUp()
    scene = Scene(setup)
    scene.shelf_4s.set_pose([0, 0, 0, 0, 0, 0])
    scene.add_shelf_4s()
    jm = JsonManager(PathRegistry.poses_json_fullpath)

    robot = scene.robot

    RobotPoseRegistry.form_poses()

    wall3_q5_t6_after_top_recoil_pose = RobotPoseRegistry.wall3_q5_t6_after_top_recoil_pose.pose_list
    robot.move_cartesian(wall3_q5_t6_after_top_recoil_pose, planner='LazyPRMstar')

    # drop_pose = RobotPoseRegistry.drop_pose.pose_list
    # trajectory = robot.move_cartesian(drop_pose, planner='LazyPRMstar')

    # home_pose = RobotPoseRegistry.home_pose.pose_list
    # robot.move_cartesian(home_pose)

    # scene.add_mesh()
    # scene.add_table()
