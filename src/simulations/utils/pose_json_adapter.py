import math
import geometry_msgs.msg

from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from utils.json_manager import JsonManager


class PoseJsonAdapter:
    """
    Adapts the json registry to be used in MoveIt.
    """

    def __init__(self, json_path: str = ''):
        self.json_path = json_path
        self.json_manager: JsonManager = JsonManager(json_path)

    @staticmethod
    def adapt_to_pose_stamped(pose: list, degrees=False, axes='rxyz', frame_id='base_link') -> PoseStamped:
        """
        Transforms a pose list to a PoseStamped object.
        Generally used for scene objects. For robot pose adaptation,
        use the adapt_robot_to_pose_stamped method.
        MoveGroup class uses this as an argument some of its methods.
        """
        pose_msg = geometry_msgs.msg.PoseStamped()

        if degrees:
            pose[3] = math.radians(pose[3])
            pose[4] = math.radians(pose[4])
            pose[5] = math.radians(pose[5])
            axes = 'sxyz'

        quaternion = quaternion_from_euler(*pose[3:], axes=axes)

        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = pose[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        return pose_msg

    @staticmethod
    def adapt_robot_to_pose_stamped(pose: list, degrees=True, axes='rxyz', frame_id='world') -> PoseStamped:
        """
        Use this method when dealing with robot's cartesian pose
        [x, y, z, roll, pitch, yaw].
        It will adapt this pose to a PoseStamped object.
        The angles should be in degrees, NOT in radian.
        """

        pose_msg = PoseJsonAdapter.adapt_to_pose_stamped(pose, degrees=degrees, axes=axes, frame_id=frame_id)

        return pose_msg

    @staticmethod
    def convert_quaternion_to_radian(pose: PoseStamped) -> tuple:
        quaternion = [0] * 4

        quaternion[0] = pose.pose.orientation.x
        quaternion[1] = pose.pose.orientation.y
        quaternion[2] = pose.pose.orientation.z
        quaternion[3] = pose.pose.orientation.w

        radian_angles = euler_from_quaternion(quaternion)

        return radian_angles

    @staticmethod
    def convert_radian_angles_to_degrees(radian_angles: tuple) -> list:
        angles = []

        for angle in radian_angles:
            degrees_angle = round(math.degrees(angle), 5)
            angles.append(degrees_angle)

        return angles

    @staticmethod
    def convert_degrees_angles_to_radian(degrees_angles: list) -> list:
        radian_angles = []

        for angle in degrees_angles:
            radian_angle = round(math.radians(angle), 5)
            radian_angles.append(radian_angle)

        return radian_angles

    @staticmethod
    def treat_degrees_angle(angle: float) -> float:
        if angle > 180:
            angle = angle - 360

        return angle

    @staticmethod
    def treat_all_degrees_angles(angles: list) -> list:
        """
        Solve the issue when an angle is sent with values above 180°.
        MoveIt only accepts angles between [-180, 180]
        """

        treated_angles = []

        for angle in angles:
            angle = PoseJsonAdapter.treat_degrees_angle(angle)
            treated_angles.append(angle)

        return treated_angles
