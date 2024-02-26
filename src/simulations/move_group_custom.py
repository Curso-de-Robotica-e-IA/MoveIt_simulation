from geometry_msgs.msg import PoseStamped
import moveit_commander
from utils.utilities import measure_duration


class MoveGroup:
    def __init__(self,
                 move_group: moveit_commander.MoveGroupCommander,
                 planner: str = "RRTConnect",
                 pose_ref_frame: str = "base_link",
                 allow_replanning: bool = False,
                 planning_attempts: int = 100,
                 planning_time: float = 2.6,
                 goal_tolerance: float = 0.025
                 ):

        self.current_joints: list = []
        self.goal_tolerance = goal_tolerance
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts
        self.allow_replanning = allow_replanning
        self.pose_ref_frame = pose_ref_frame
        self.planner = planner
        self.move_group = move_group
        self.current_pose = self.get_current_pose()

    def get_current_pose(self) -> PoseStamped:
        return self.move_group.get_current_pose()

    def get_current_joints(self) -> list:
        """
        Return:
            A list with the values of the joints angles in radian
        """
        return self.move_group.get_current_joint_values()

    def plan_and_execute_pose(self, pose: PoseStamped, planner: str = 'RRTConnect') -> any:
        """
        Plan the path from start state to goal state. These states
        are set as a pose represented by a PoseStamped object


        Args:
            pose: PoseStamped object with position(x, y, z)
             and orientation(roll, pitch, yaw)
                OBS: Supply angles in degrees

            planner: A string with the planner to use. The name needs
                     to be the same as in Rviz grapical interface.
                     Ex: 'RRTConnect'

        Return:
            ...
        """
        self.move_group.set_start_state_to_current_state()

        self.move_group.set_pose_target(pose)
        # move_group.set_goal_tolerance(0.025)
        self.set_planning_config(planner=planner)
        print("planner query id -- ", self.move_group.get_planner_id())
        plan = self.move_group.plan()

        if not MoveGroup.plan_is_successful(plan):
            return

        print("going to position")
        with measure_duration():
            success = self.move_group.execute(plan[1], wait=True)
            print("result of move_group execute:", success)
            print("trajectory type:", type(plan[1]))

        if not success:
            return

        self.current_pose = self.move_group.get_current_pose()
        # print("current pose\n", self.current_pose)

        return plan[1]

    def plan_and_execute_joints(self, joints: list, planner: str = 'RRTConnect'):
        """
        Plan the path from start state to goal state. These states
        are set as joints angles.


        Args:
            joints: A list of robot's joints angles with degrees values
                OBS: Supply angles in degrees

            planner: A string with the planner to use. The name needs
                     to be the same as in Rviz grapical interface.
                     Ex: 'LazyPRMstar'

        Return:
            ...
        """
        self.move_group.set_start_state_to_current_state()

        self.move_group.set_joint_value_target(joints)
        # move_group.set_goal_tolerance(0.025)
        self.set_planning_config(planner=planner)
        print("planner query id -- ", self.move_group.get_planner_id())
        plan = self.move_group.plan()

        if not MoveGroup.plan_is_successful(plan):
            return

        print("going to position")
        with measure_duration():
            success = self.move_group.execute(plan[1], wait=True)

        if not success:
            return

        self.current_joints = self.move_group.get_current_joint_values()
        print("current joints values\n", self.current_joints)

    def is_same_pose(self, pose_start, pose_goal):
        ...

    def set_planning_config(self,
                            planner: str = "RRTConnect",
                            pose_ref_frame: str = "base_link",
                            allow_replanning: bool = False,
                            planning_attempts: int = 50,
                            planning_time: float = 3.0,
                            goal_tolerance: float = 0.025,
                            ):
        """
        Set configuration to use in planning scenes. Such as:

        planner: Planner to be used. Set it as a string
        equal to the name in rviz graphical interface

        pose_ref_frame: Reference frame inside PoseStamped object

        planning_attempts: Tries till timeout

        planning_time: Time till timeout. The timeout occurs when planning_attempts or planning time is reached

        goal_tolerance: Helps when planning is consistently failing, but raises collision probability

        allow_replanning:

        Default configuration:

        planner: str = "RRTConnect",
        pose_ref_frame: str = "base_link",
        allow_replanning: bool = False,
        planning_attempts: int = 100,
        planning_time: float = 2.6,
        goal_tolerance: float = 0.025 (not allowed by default)

        """
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts
        self.allow_replanning = allow_replanning
        self.pose_ref_frame = pose_ref_frame
        self.planner = planner

        self.move_group.set_planner_id(self.planner)
        self.move_group.set_pose_reference_frame(self.pose_ref_frame)
        self.move_group.allow_replanning(self.allow_replanning)
        self.move_group.set_num_planning_attempts(self.planning_attempts)
        self.move_group.set_planning_time(self.planning_time)

        # Parâmetro para melhorar a taxa de sucesso,
        # porém, aumentando as chances de colisão.
        # self.move_group.set_goal_tolerance(0.025)

    @staticmethod
    def plan_is_successful(plan: tuple):
        """
        Args:
             plan (tuple): A tuple with the following elements:
                (MoveItErrorCodes, trajectory_msg, planning_time, error_code)

        Returns:
            bool: True if plan successfully computed.
        """

        print("plan success", plan[0])
        return plan[0]
