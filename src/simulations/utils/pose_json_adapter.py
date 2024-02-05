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
    def adapt_robot_to_pose_stamped(pose: list, degrees=False, axes='rxyz', frame_id='world') -> PoseStamped:
        """
        Transforms a pose list to a PoseStamped object. The MoveGroup from MoveIt
        uses this object as argument for some of its methods.
        """

        pose_msg = PoseJsonAdapter.adapt_to_pose_stamped(pose, degrees=True, axes='sxyz', frame_id='world')

        return pose_msg

    @staticmethod
    def adapt_quaternion_to_radian(pose: PoseStamped) -> tuple:
        quaternion = [0] * 4

        quaternion[0] = pose.pose.orientation.x
        quaternion[1] = pose.pose.orientation.y
        quaternion[2] = pose.pose.orientation.z
        quaternion[3] = pose.pose.orientation.w

        radian_angles = euler_from_quaternion(quaternion)

        return radian_angles

    @staticmethod
    def adapt_radian_angles_to_degrees(radian_angles: tuple) -> list:
        angles = []

        for angle in radian_angles:
            degrees_angle = round(math.degrees(angle), 5)
            angles.append(degrees_angle)

        return angles
