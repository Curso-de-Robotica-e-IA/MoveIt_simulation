import moveit_commander
import rospy


class SetUp:

    def __init__(self):
        SetUp.show_init_log()

        SetUp.create_node()

        SetUp.initialize_topic()

    @staticmethod
    def create_node():
        rospy.init_node('planning_scene_kinova', anonymous=True, log_level=rospy.DEBUG)

    @staticmethod
    def show_init_log():
        rospy.loginfo('Starting the Scene SetUp')

    @staticmethod
    def initialize_topic():
        joint_state_topic = ['joint_states:=/my_gen3_lite/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)

    @staticmethod
    def get_robot_commander() -> moveit_commander.RobotCommander:
        robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
        return robot

    @staticmethod
    def get_scene() -> moveit_commander.PlanningSceneInterface:
        scene = moveit_commander.PlanningSceneInterface('my_gen3_lite', synchronous=True)
        return scene

    @staticmethod
    def get_move_group() -> moveit_commander.MoveGroupCommander:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name,
                                                         robot_description="/my_gen3_lite/robot_description",
                                                         ns="my_gen3_lite")
        return move_group
