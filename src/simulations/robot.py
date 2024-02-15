from geometry_msgs.msg import PoseStamped
from move_group_custom import MoveGroup
from utils.pose_json_adapter import PoseJsonAdapter


class Robot:
    def __init__(self, move_group: MoveGroup):
        self.move_group = move_group
        self.pose_stamped: PoseStamped = self.get_pose()
        self.joints: list = self.get_joints_radian()
        self.joints_degrees: list = self.get_joints()

    def get_pose(self) -> PoseStamped:
        return self.move_group.move_group.get_current_pose()

    def get_pose_radian_angles(self) -> tuple:
        self.pose_stamped = self.get_pose()

        orientation_angles = PoseJsonAdapter.convert_quaternion_to_radian(self.pose_stamped)

        return orientation_angles

    def get_cartesian(self) -> list:
        orientation_angles = self.get_pose_radian_angles()

        # A pose do kinova tem 3 valores de posição x, y, z, e
        # 3 valores de orientação adicionadas pelo método append
        pose_kortex = [0] * 3

        pose_kortex[0] = round(self.pose_stamped.pose.position.x, 5)
        pose_kortex[1] = round(self.pose_stamped.pose.position.y, 5)
        pose_kortex[2] = round(self.pose_stamped.pose.position.z, 5)

        angles = PoseJsonAdapter.convert_radian_angles_to_degrees(orientation_angles)

        pose_kortex.extend(angles)

        return pose_kortex

    def get_joints_radian(self):
        return self.move_group.get_current_joints()

    def get_joints(self):
        self.joints = self.get_joints_radian()
        self.joints_degrees = PoseJsonAdapter.convert_radian_angles_to_degrees(tuple(self.joints))

        return self.joints_degrees

    def move_cartesian(self, pose: list, planner: str = 'LazyPRMstar'):
        """
        Move the robot in Rviz graphical interface using pose values

        Args:
            pose: A list representing a pose in this format: [x, y, z, roll, pitch, yaw]
                OBS: Supply angles in degrees

            planner: A string with the planner to use. The name needs
                     to be the same as in Rviz grapical interface.
                     Ex: 'RRTConnect'

        """
        pose_stamped = PoseJsonAdapter.adapt_robot_to_pose_stamped(pose)

        self.move_group.plan_and_execute_pose(pose_stamped, planner=planner)

    def move_joints(self, joints: list, planner: str = 'LazyPRMstar'):
        """
        Move the robot in Rviz graphical interface using joints values

        Args:
            joints: A list with joints values. Angles have to be in degrees

            planner: A string with the planner to use. The name needs
                     to be the same as in Rviz grapical interface.
                     Ex: 'RRTConnect'

        """
        print("joint angles:", joints)
        joints = PoseJsonAdapter.treat_all_degrees_angles(joints)
        print("treated joint angles:", joints)
        joints_radian = PoseJsonAdapter.convert_degrees_angles_to_radian(joints)
        print("radian angles:", joints_radian)

        self.move_group.plan_and_execute_joints(joints_radian, planner=planner)

    def attach_support(self):
        self.move_group.move_group.attach_object('support', 'gripper_base_link')

