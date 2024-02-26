from geometry_msgs.msg import PoseStamped
from utils.pose_json_adapter import PoseJsonAdapter
import geometry_msgs.msg


class SceneObject:
    def __init__(self, name: str = ''):
        self.name = name
        self.pose_msg = geometry_msgs.msg.PoseStamped()

    def set_pose(self, pose: list, degrees=False, frame_id: str = 'base_link'):
        self.pose_msg = PoseJsonAdapter.adapt_to_pose_stamped(pose, degrees=degrees, frame_id=frame_id)

    def change_pose(self, pose: list):
        self.set_pose(pose)

    def get_pose(self) -> geometry_msgs.msg.PoseStamped:
        return self.pose_msg


class ShelfOnly3s(SceneObject):
    def __init__(self, name: str = 'shelf_only_3s'):
        super().__init__(name)
        self.initial_pose: list = [0.35, 1.29, 0.01, 1.60, 0.50, 0.00]
        self.set_pose(self.initial_pose)


class Shelf4s(SceneObject):
    def __init__(self, name: str = 'shelf_4s'):
        super().__init__(name)
        self.initial_pose: list = [0.00, 0.00, -0.14, 0.00, 0.00, 0.00]
        self.set_pose(self.initial_pose)


class Support(SceneObject):
    def __init__(self, name: str = 'support'):
        super().__init__(name)
        self.initial_pose: list = [0.0, 0.0, 0.0, 0.0, 0.0, 90]
        self.set_pose(self.initial_pose, degrees=True, frame_id='gripper_base_link')


class TableObject(SceneObject):
    def __init__(self, name: str = 'Table_box'):
        super().__init__(name)
        self.set_pose()

    def set_pose(self, pose: list = [0.00, 0.00, -0.01]):
        self.pose_msg.header.frame_id = "base_link"
        self.pose_msg.pose.position.x = pose[0]
        self.pose_msg.pose.position.y = pose[1]
        self.pose_msg.pose.position.z = pose[2]
