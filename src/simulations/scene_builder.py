from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, RobotState
from set_up import SetUp
from move_group_custom import MoveGroup
from robot import Robot
from scene_object import ShelfOnly3s, TableObject, Shelf4s, Support
from utils.pose_json_adapter import PoseJsonAdapter
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import rospy
from utils.json_manager import JsonManager


class Scene:
    def __init__(self, setup: SetUp):
        self.objects_name = None
        self.scene_pub: rospy.Publisher = None

        self.setup = setup

        self.scene = self.setup.get_scene()
        self.move_group = self.setup.get_move_group()

        self.mg = MoveGroup(self.move_group)
        self.robot = Robot(self.mg)

        self.shelf_only_3s = ShelfOnly3s()
        self.table_obj = TableObject()
        self.shelf_4s = Shelf4s()
        self.support = Support()

        self.add_support()
        self.attach_support_to_robot()

    def set_scene_publisher(self):
        self.scene_pub = rospy.Publisher('/my_gen3_lite/planning_scene', moveit_msgs.msg.PlanningScene, queue_size=20)

    def clean_scene(self):
        rospy.loginfo('Cleaning of the objects in the scene')
        try:
            self.scene.clear()

        except Exception as e:
            print(e)

    def change_shelf_4s_pose(self, name_id: str, height):
        # TODO - Fazer um método geral que cada objeto vai utilizar específico para cada um
        """
        Change shelf position, using an inherited implementation
        of PlanningSceneInterface class and CollisionObject message class.
        PlanningSceneInterface: https://docs.ros.org/en/jade/api/moveit_commander/html/planning__scene__interface_8py_source.html
        CollisionObject MSG: https://docs.ros.org/en/api/moveit_msgs/html/msg/CollisionObject.html
        """
        obj = self.scene.get_objects([name_id])
        obj = obj[name_id]
        obj.pose.position.z = -height
        obj.operation = CollisionObject.MOVE

        print("scene methods:\n", dir(self.scene))
        req = self.scene._PlanningSceneInterface__make_planning_scene_diff_req(obj)
        self.scene._apply_planning_scene_diff(req)

    def add_shelf_only_3s(self,
                          mesh_path: str = '/home/rcmm/Documents/kinova_ros/catkin_workspace/src/testes_estantes/scripts/teste3oandarmodelo3d/montagemPlanejamento-3andares.stl'):
        """
        Adds a file mesh to a MoveIt scene.
        """
        rospy.loginfo(f'Adding the structure: {self.shelf_only_3s.name}')

        self.scene.add_mesh(
            self.shelf_only_3s.name,
            self.shelf_only_3s.get_pose(),
            mesh_path
        )

    def add_shelf_4s(self,
                     mesh_path: str = '/home/rcmm/Downloads/montagemPlanejamento-Completa-4andares-wMesa-modif.stl'):
        """
        Adds a file mesh to a MoveIt scene.
        """
        rospy.loginfo(f'Adding the structure: {self.shelf_4s.name}')

        self.scene.add_mesh(
            self.shelf_4s.name,
            self.shelf_4s.get_pose(),
            mesh_path
        )

    def add_support(self,
                    mesh_path: str = '/home/rcmm/Documents/kinova_ros/catkin_workspace/src/MoveIt_simulation/src/simulations/models3D/montagemD405-modif.stl'):
        """
        Adds the support with the camera and bomb cylinder.
        """

        rospy.loginfo(f'Adding the structure: {self.support.name}')

        self.scene.add_mesh(
            self.support.name,
            self.support.get_pose(),
            mesh_path
        )

    def set_shelf_only_3s_pose(self, pose: list, name: str = ''):
        self.shelf_only_3s.set_pose(pose, name)

    def add_table(self, size: tuple = (1.90, 1.8, 0.01)):
        rospy.loginfo('Adding the table object')
        self.scene.add_box(self.table_obj.name, self.table_obj.pose_msg, size=size)

    def get_objects(self) -> list:
        self.objects_name = self.scene.get_known_object_names()
        return self.objects_name

    def remove_object(self, name: str):
        self.scene.remove_world_object(name)

    def attach_support_to_robot(self):
        self.robot.attach_support()


class PathPlanner:
    def __init__(self):
        self.planned_path_pub: rospy.Publisher = None

    def set_path_publisher(self):
        self.planned_path_pub = rospy.Publisher("/move_group/display_planned_path",
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20,
                                                )


if __name__ == "__main__":
    setup = SetUp()
    scene = Scene(setup)
    print(scene.table_obj.name)
    scene.clean_scene()
    jm = JsonManager()
    # scene.add_mesh()
    # scene.add_table()
