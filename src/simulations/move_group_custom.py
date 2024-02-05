from geometry_msgs.msg import PoseStamped
import moveit_commander


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

    def plan_and_execute(self, pose: PoseStamped, planner: str = ''):
        """
        Plan the path from start state to goal state.
        First checks if the robot is in start state,
        in case not, move the robot to start state
        and then plans the trajectory to goal.

        Args:
            pose: List with pose to go. Ex: [x, y, z, roll, pitch, yaw]
                OBS: Supply angles in degrees
            planner: A string with the planner to use. The name needs
                     to be the same as in Rviz grapical interface.
                     Ex: 'LazyPRMstar'

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
        success = self.move_group.execute(plan[1], wait=True)

        if not success:
            return

        self.current_pose = self.move_group.get_current_pose()
        print("current pose\n", self.current_pose)

        self.move_group.set_start_state_to_current_state()

    def is_same_pose(self, pose_start, pose_goal):
        ...

    def set_planning_config(self,
                            planner: str = "RRTConnect",
                            pose_ref_frame: str = "base_link",
                            allow_replanning: bool = False,
                            planning_attempts: int = 100,
                            planning_time: float = 2.6,
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
