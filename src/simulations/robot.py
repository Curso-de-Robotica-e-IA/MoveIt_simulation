from geometry_msgs.msg import PoseStamped
from move_group_custom import MoveGroup
from utils.pose_json_adapter import PoseJsonAdapter


class Robot:
    def __init__(self, move_group: MoveGroup):
        self.move_group = move_group
        self.pose_stamped: PoseStamped = self.get_pose()

    def get_pose(self) -> PoseStamped:
        return self.move_group.move_group.get_current_pose()

    def get_radian_angles(self) -> tuple:
        self.pose_stamped = self.get_pose()

        orientation_angles = PoseJsonAdapter.adapt_quaternion_to_radian(self.pose_stamped)

        return orientation_angles

    def get_cartesian(self) -> list:
        orientation_angles = self.get_radian_angles()

        # A pose do kinova tem 3 valores de posição x, y, z, e
        # 3 valores de orientação adicionadas pelo método append
        pose_kortex = [0] * 3

        pose_kortex[0] = round(self.pose_stamped.pose.position.x, 5)
        pose_kortex[1] = round(self.pose_stamped.pose.position.y, 5)
        pose_kortex[2] = round(self.pose_stamped.pose.position.z, 5)

        angles = PoseJsonAdapter.adapt_radian_angles_to_degrees(orientation_angles)

        pose_kortex.extend(angles)

        return pose_kortex

    def move(self, pose, planner: str = 'LazyPRMstar'):
        pose_stamped = PoseJsonAdapter.adapt_robot_to_pose_stamped(pose)

        self.move_group.plan_and_execute(pose_stamped, planner=planner)

    def attach_support(self):
        self.move_group.move_group.attach_object('support', 'gripper_base_link')

