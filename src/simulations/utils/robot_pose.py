
class RobotPose:
    
    def __init__(self, pose_name: str, pose_list: list):
        self.pose_name = pose_name
        self.pose_list = pose_list
    
    def __name__(self) -> str:
        return self.pose_name

    def __repr__(self) -> str:
        return self.pose_name


if __name__ == "__main__":
    pose_l = [1, 2]
    pose1 = RobotPose("pose1", pose_l)

    pose1_dict = {pose1: pose1.pose_list}
    print(pose1_dict)
    print(pose1)