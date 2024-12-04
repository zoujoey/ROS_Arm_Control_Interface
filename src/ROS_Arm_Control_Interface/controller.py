import rospy
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
import franka_gripper.msg as frg 
from unity_robotics_demo_msgs.msg import PosRot
from std_msgs.msg import Float64MultiArray
import copy
import math
import numpy as np
from sensor_msgs.msg import JointState as RosJointState
from ROS_Arm_Control_Interface.msg import StateInfo, ControllerInfo, ButtonState
from ROS_Arm_Control_Interface.msg import JointState as UnityJointState
from actionlib_msgs.msg import GoalID
import actionlib
from .types import ControlMode, InterfaceMode, InputButtonXR
from .logger import ThreadedPublisher
from . import topics

class Controller:
    """Provides arm state and control

    Acts as intermediary between the state machine, the arm, and XR interface by using frankaros topics and messages.

    Will block indefinitely on initialization if frankaros is not active.

    Controller has three modes:
    1. Manual Position - the controller cube is used to control the end-effector posiiton only, fully controlled by user
    2. Manual Orientation - the controller cube is used to control the end-effector pose and orientation, fully controlled by user
    3. Automatic - some autonomous script is used to control the arm
    4. End - arm has reached the target goal, no control commands are sent

    Attributes (Frames are all defined by the initial position):
        control_mode (ControlMode): current state of the controller
        ee_pose_cur (List(7)): current pose of the end effector
        ee_pose_target (List(7)): target pose to be published to the end effector
        ee_pose_default (List(7)): initial and default pose of the end effector
        cc_pose_cur (List(7)): current pose of the controller cube
        cc_pose_init (List(7)): initial pose of the controller cube
        rg_pose_cur (Float): current pose of robot gripper
        rg_pose_target (Float): target pose of robot gripper
        rg_pose_default (Float): default pose of robot gripper
        joint_angles (List(7)): joint angles of robot arm
        gripper_angles (List(2)): joint angles of robot gripper

        position_tolerance (List(6)): tolerance in position x,y,z for end-effector setpoint control 
        orientation_tolerance (Float): tolerance in orientation for end-effector setpoint control
        bounding_box (List(6)): [-x, x, -y, y, -z, z] maxmimum values for the bounding box dimensions

    Properites:
        control_mode -> ControlMode:
        interface_mode -> InterfaceMode:
    
    Control Methods:
        set_target_position_cc -> None:
        set_target_position_ee -> None:
        set_target_position_dp -> None:

        has_arrived_position -> bool:
        has_arrived_orientation -> bool:
        has_arrived_target -> bool:
        
        manual_pos_mode -> None:
        manual_ori_mode -> None:
        automatic_mode -> None:
        finished -> None:
        exit -> None:

    Callback Methods:
        cc_pose_callback -> None:
        ee_pose_callback -> None:
        joint_state_callback -> None:
        print_state -> None:

    Utility Methods:
        position_comparison -> bool:
        convert_to_list -> None:
        coordinate_transform_Unity_to_ROS -> List(3):
        quaternion_inverse -> List(4):
        quaternion_multiply -> List(4):
        get_relative_pose -> List(7):
        apply_transformation -> List(7):
        initialize_connection -> None:
        get_interface_mode -> InterfaceMode: 
        




    """
    def __init__(self):
        """Creates a Controller instance

        call initialize_connection() to connect to VR interface

        """

    ######################### Initialize Variables ############################
        # Arm Controller State
        self._connection_initialized = False
        self._control_mode = ControlMode.FINISHED
        self._interface_mode = self.get_interface_mode()
        self._target_publisher = TargetPublisher(self)
        self._controller_info_publisher = ControllerInfoPublisher(self)
        #End Effector Variables
        self.ee_pose_cur = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #Callback
        self.ee_pose_target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ee_pose_default = rospy.get_param("ee_pose_default") #Parameter
        
        #Controller Cube Variables
        self.cc_pose_cur = [100, 100, 100, 100, 100, 100, 100]
        self.cc_pose_init = [0, 0, 0, 0, 0, 0, 0]

        #Robot Gripper Variables
        self.rg_pose_cur = 0.0
        self.rg_pose_target = 0.0
        self.rg_pose_default = rospy.get_param("rg_pose_default")
        self.rg_speed_target = 0.0
        self.rg_speed_default = rospy.get_param("rg_speed_default")
        self.rg_force_target = 0
        self.rg_force_default = rospy.get_param("rg_force_default")
        
        #Arm Joint Variables
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        self.gripper_angles = [0.0, 0.0]

        #Button Variables
        self.button_input = InputButtonXR()

        #Control Parameters
        self.position_tolerance = rospy.get_param("position_tolerance")
        self.orientation_tolerance = rospy.get_param("orientation_tolerance")
        self.bounding_box = rospy.get_param("bounding_box")
        self.rg_tolerance = rospy.get_param("rg_position_tolerance")
        
        self.rate = rospy.get_param("ros_rate")
        self.unity = rospy.get_param("unity_controller")
        
        # State Logger Publisher
        self.state_logger_pub = rospy.Publisher(topics.STATE_INFO, StateInfo, queue_size=10)
        
        # Initialize Publishers and Subscribers (Robot)
        self.ee_pose_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
        self.ee_pose_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.ee_pose_callback)
        self.rg_pose_sub = rospy.Subscriber('/franka_gripper/joint_states', RosJointState, self.rg_joint_state_callback)
        self.ros_joint_state_sub = rospy.Subscriber('/franka_state_controller/joint_states', RosJointState, self.joint_state_callback)
        
        # Initialize Publishers and Subscribers (Unity)
        self.cc_pose_sub = rospy.Subscriber('/ccube_position', PosRot, self.cc_pose_callback)
        self.input_pose_sub = rospy.Subscriber('/button_state', ButtonState, self.button_state_callback)

        self.gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', frg.GraspAction)
        self.gripper_stop_client = actionlib.SimpleActionClient('/franka_gripper/stop', frg.StopAction)
        self.gripper_move_client = actionlib.SimpleActionClient('/franka_gripper/move', frg.MoveAction)
        print("waiting for gripper servers")
        self.gripper_grasp_client.wait_for_server()
        self.gripper_stop_client.wait_for_server()
        self.gripper_move_client.wait_for_server()
        print("gripper servers online")
        self._rate = rospy.Rate(self.rate)  # never call in methods intended to run in event loop

    ######################## Properties ####################################
    @property
    def control_mode(self) -> ControlMode:
        """Current mode of the controller

        Returns the private control_mode member to disallow external modification except by member functions

        Returns:
            ControlMode: Current mode enum
        """
        return self._control_mode
    
    @property
    def interface_mode(self) -> InterfaceMode:
        """Interface mode (set by launch file)

        Returns the private interface_mode member to disallow modification

        Returns:
            InterfaceMode: interface mode of control
        """
        return self._interface_mode

    ######################## Control Methods ###############################
    
    def set_target_position_cc(self):
        relative_cc_pose = self.get_relative_pose(self.cc_pose_init, self.cc_pose_cur)
        self.set_target_position_ee(relative_cc_pose)      

    def set_default_ee_position(self):
        self.set_target_position_ee()
        return True
    
    def set_target_position_rg(self, target=None, speed = None, force = None):
        self.rg_pose_target = target if target is not None else self.rg_pose_default
        self.rg_speed_target = speed if speed is not None else self.rg_speed_default
        self.rg_force_target = force if force is not None else self.rg_force_default
        
    def set_target_position_ee(self, transformation=None, base_pose=None):
        """
        Set the target end-effector position by applying a transformation to a start pose.

        transformation: list of 7 elements [x, y, z, qx, qy, qz, qw]
        start_pose: list of 7 elements [x, y, z, qx, qy, qz, qw]
        """
        # Use default values if not provided
        base_pose = copy.deepcopy(base_pose) if base_pose is not None else copy.deepcopy(self.ee_pose_default)
        transformation = copy.deepcopy(transformation) if transformation is not None else [0, 0, 0, 0, 0, 0, 1]
        if self._control_mode == ControlMode.MANUAL_POSITION:
            transformation = copy.deepcopy(transformation[:3] + [0, 0, 0, 1]) if transformation is not None else [0, 0, 0, 0, 0, 0, 1]
        # Apply the transformation to the start pose
        target_pose = copy.deepcopy(self.apply_transformation(base_pose, transformation))
        #SAFETY CHECK WITH BOUNDING SPHERE
        if self.position_comparison(self.ee_pose_default, target_pose, self.bounding_box):
            self.ee_pose_target = target_pose
        else:
            print("ERROR: OUTSIDE BOUNDING BOX")

    def publish_ee_pose(self):
        """Publishes the pose to the Cartesian Impedance Controller."""
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'  # Adjust to your robot's base frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.ee_pose_target[0]
        msg.pose.position.y = self.ee_pose_target[1]
        msg.pose.position.z = self.ee_pose_target[2]
        # Normalize the quaternion
        normalized_quaternion = self.normalize_quaternion(self.ee_pose_target[3:])

        # Assign the normalized quaternion to the message
        msg.pose.orientation.x = normalized_quaternion[0]
        msg.pose.orientation.y = normalized_quaternion[1]
        msg.pose.orientation.z = normalized_quaternion[2]
        msg.pose.orientation.w = normalized_quaternion[3]

        # Publish the message        
        self.ee_pose_pub.publish(msg)
    
    def publish_rg_grasp(self):
        goal = frg.GraspGoal()
        goal.width = self.rg_pose_target
        goal.epsilon.inner = self.rg_tolerance
        goal.epsilon.outer = self.rg_tolerance
        goal.speed = self.rg_speed_target
        goal.force = self.rg_force_target
        self.gripper_grasp_client.send_goal(goal)
        self.gripper_grasp_client.wait_for_result()
    
    def publish_rg_move(self):
        goal = frg.MoveGoal()
        goal.width = self.rg_pose_target
        goal.speed = self.rg_speed_target
        self.gripper_move_client.send_goal(goal)
        self.gripper_move_client.wait_for_result()

    def publish_rg_stop(self):
        goal = frg.StopGoal()
        self.gripper_stop_client.send_goal(goal)
        self.gripper_stop_client.wait_for_result()


    ######################## Callback Methods ###############################
    def cc_pose_callback(self, msg):
        """Callback function for the controller_pos_rot topic."""
        # Compute the difference between the initial and current positions
        #Transformation from Unity to ROS coordinates
        new_pose = copy.deepcopy(self.coordinate_transform_Unity_to_ROS([msg.pos_x, msg.pos_y, msg.pos_z]))
        if self._control_mode == ControlMode.MANUAL_ORIENTATION:
            self.cc_pose_cur = [new_pose[0], new_pose[1], new_pose[2], 
                            msg.rot_x, msg.rot_y, msg.rot_z, msg.rot_w]
        else:
            self.cc_pose_cur = [new_pose[0], new_pose[1], new_pose[2], 
                            self.ee_pose_default[3], self.ee_pose_default[4], self.ee_pose_default[5], self.ee_pose_default[6]]
    
    def ee_pose_callback(self, msg):
        """Callback function for the franka_state_controller/franka_states topic"""
        # Update the current End Effector position
        self.ee_pose_cur = [msg.O_T_EE[12], msg.O_T_EE[13], msg.O_T_EE[14],
                            msg.O_T_EE[0], msg.O_T_EE[1], msg.O_T_EE[2], msg.O_T_EE[3]]

    def rg_joint_state_callback(self, msg):
        self.rg_pose_cur = msg.position[0]
        self.gripper_angles = copy.deepcopy(msg.position)
    
    def joint_state_callback(self, data):
        # Extract the joint positions from the JointState message
        self.joint_angles = copy.deepcopy(data.position)  # assuming this is a tuple or list

    def button_state_callback(self, state):

        self.button_input.grab = state.grab
        self.button_input.index = state.index
        self.button_input.stick = state.stick
        self.button_input.x_input = state.x_input
        self.button_input.y_input = state.y_input
        self.button_input.stick_up = state.stick_up
        self.button_input.stick_down = state.stick_down

    def get_controller_info(self):
        """Prints the current status of all relevant variables."""
        def format_pose(pose):
            if pose is None:
                return "None"
            return "x={:.6f}, y={:.6f}, z={:.6f}, \n qx={:.6f}, qy={:.6f}, qz={:.6f}, qw={:.6f}".format(
                pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]
            )
        status = ("Publishing Pose:\n"
            "Target Position: " + format_pose(self.ee_pose_target) + "\n"
            "Current EE Position: " + format_pose(self.ee_pose_cur) + "\n"
            "Initial EE Position: " + format_pose(self.ee_pose_default if self.ee_pose_default else None) + "\n"
            "Initial CC Position: " + format_pose(self.cc_pose_init if self.cc_pose_init else None) + "\n"
            "Current CC Position: " + format_pose(self.cc_pose_cur if self.cc_pose_cur else None) + "\n"
            "Current RG Position: " + str(self.gripper_angles) + "\n"
            "Target RG Position: " + str(self.rg_pose_target) + "\n"
            "Joint Angles: " + str(self.joint_angles)
        )
        # print(status)
        return status
    
    
    ######################## Utility Functions ################################
    def has_gripped_target(self):
        return self.gripper_comparison([self.rg_pose_target/2, self.rg_pose_target/2], self.gripper_angles)
    
    def gripper_comparison(self, target, current, gripper_tolerance = None):
        gripper_tolerance = gripper_tolerance if gripper_tolerance is not None else self.rg_tolerance
        for i in range(2):
            if abs(current[i]- target[i]) > gripper_tolerance:
                return False
        return True
    
    def has_arrived_target(self):
        return self.position_comparison(self.ee_pose_target,self.ee_pose_cur)
    
    def position_comparison(self, target, current, pose_tolerance = None, orientation_tolerance = None):
        """Checks if the current EE position is in the default position."""
        pt = pose_tolerance if pose_tolerance is not None else self.position_tolerance
        ot = orientation_tolerance if orientation_tolerance is not None else self.orientation_tolerance
        for i in range(3):
            if (current[i] - target[i]) < pt[2*i] or (current[i] - target[i]) > pt[2*i+1]:
                return False
        for i in range(3,7):
            if abs(target[i] - current[i]) > ot:
                return False
        return True
    
    def convert_to_list(self, msg):
        """Converts position and orientation data from different message types to a list of 7 elements."""
        if isinstance(msg, PoseStamped):
            return copy.deepcopy([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        elif isinstance(msg, PosRot):
            return copy.deepcopy([msg.pos_x, msg.pos_y, msg.pos_z,
                msg.rot_x, msg.rot_y, msg.rot_z, msg.rot_w])
        else:
            print("Unsupported message type for conversion.")
            return None
    
    def coordinate_transform_Unity_to_ROS(self, pose):
        new_pose = [-pose[0],-pose[2],pose[1]]
        return new_pose
    
    def quaternion_inverse(self, quaternion):
        """Returns the inverse of a quaternion."""
        q_conjugate = np.array([-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]])
        norm_squared = np.dot(quaternion, quaternion)
        return q_conjugate / norm_squared

    def quaternion_multiply(self, q1, q2):
        """Multiplies two quaternions."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        ])
    
    def normalize_quaternion(self, quaternion):
        """Normalizes a quaternion.

        Args:
            quaternion: A list or tuple of four elements representing the quaternion (x, y, z, w).

        Returns:
            A normalized quaternion.
        """

        magnitude = math.sqrt(quaternion[0]**2 + quaternion[1]**2 + quaternion[2]**2 + quaternion[3]**2)
        if magnitude == 0:
            return quaternion  # Avoid division by zero

        normalized_quaternion = [q / magnitude for q in quaternion]
        return normalized_quaternion

    def get_relative_pose(self, pose_A, pose_B):
        """
        Calculate the relative pose between pose_A and pose_B.
        
        pose_A: list of 7 elements [x, y, z, qx, qy, qz, qw]
        pose_B: list of 7 elements [x, y, z, qx, qy, qz, qw]
        
        Returns:
        A list representing the relative pose [rel_x, rel_y, rel_z, rel_qx, rel_qy, rel_qz, rel_qw]
        """
        # Extract position and orientation
        pos_A, ori_A = np.array(pose_A[:3]), np.array(pose_A[3:])
        pos_B, ori_B = np.array(pose_B[:3]), np.array(pose_B[3:])
        
        # Calculate relative position
        rel_pos = pos_B - pos_A
        
        # Calculate relative orientation
        ori_A_inv = self.quaternion_inverse(ori_A)
        rel_ori = self.quaternion_multiply(ori_B, ori_A_inv)
        
        # Combine relative position and orientation into a single list
        relative_pose = np.concatenate((rel_pos, rel_ori))
        
        return relative_pose.tolist()
    
    def apply_transformation(self, base_pose, transformation):
        """
        Apply a transformation to a base pose.

        base_pose: list of 7 elements [x, y, z, qx, qy, qz, qw]
        transformation: list of 7 elements [x, y, z, qx, qy, qz, qw]
        
        Returns:
        Transformed pose as a list [x, y, z, qx, qy, qz, qw]
        """
        # Extract position and orientation from the base pose and transformation
        pos_base, ori_base = np.array(base_pose[:3]), np.array(base_pose[3:])
        pos_trans, ori_trans = np.array(transformation[:3]), np.array(transformation[3:])
        
        # Calculate transformed position
        transformed_pos = pos_base + pos_trans
        
        # Calculate transformed orientation
        transformed_ori = self.quaternion_multiply(ori_base, ori_trans)
        
        # Combine transformed position and orientation into a single list
        transformed_pose = np.concatenate((transformed_pos, transformed_ori))
        
        return transformed_pose.tolist()
    
    def initialize_connection(self):
        """Initializes controller variables, does not enter OFFBOARD or arm the controller

        Call init_default_ee_position() to reset the arm_position

        Call enter_offboard() to enter OFFBOARD mode

        """
        # Set Initial End Effector Position
        print("Waiting for Robot State Subscription") 
        while ((not rospy.is_shutdown()) and self.ee_pose_cur == None): 
            self._rate.sleep()
        self.set_default_ee_position()
        self._target_publisher.start()
        self._controller_info_publisher.start()
        while ((not rospy.is_shutdown()) and (not self.has_arrived_target())): 
            self._rate.sleep()

        print("Completed Reset Robot to Default Position")
        self.init_cube_controller()
        print("Controller Cube Subscription Established")
        self._connection_initialized = True
    
    def init_cube_controller(self):
        print("Waiting for Hand Controller Subscription") 
        while (((not rospy.is_shutdown()) and self.cc_pose_cur == [100, 100, 100, 100, 100, 100, 100]) and self.unity):
            rospy.sleep(5)
            print(self.cc_pose_cur)
        self.cc_pose_init = copy.deepcopy(self.cc_pose_cur)
    

    @staticmethod
    def get_interface_mode() -> InterfaceMode:
        """Gets current interface mode from ROS param server

        Raises:
            KeyError: if rosparm interface_mode is not set

        Returns:
            InterfaceMode: interface mode struct
        """
        try:
            interface_mode = rospy.get_param("interface_mode")
        except KeyError as no_key:
            raise no_key

        mode = interface_mode
        is_simulation = False
        is_hardware = False
        is_real = False

        if interface_mode == 'simulation':
            is_simulation = True
        elif interface_mode == 'hardware':
            is_hardware = True
        elif interface_mode == 'real':
            is_real = True
        else:
            raise KeyError('rosparam "interface_mode" is an invalid value')

        return InterfaceMode(mode=mode, is_simulation=is_simulation, is_hardware=is_hardware, is_real=is_real)
    
    def manual_position_mode(self) -> None:
        """Sets arm to manually controlled position mode, 
        """
        self._control_mode = ControlMode.MANUAL_POSITION

    def manual_orientation_mode(self) -> None:
        """Sets arm to manually controlled position and orientation mode, 
        """
        self._control_mode = ControlMode.MANUAL_ORIENTATION

    def automatic_mode(self) -> None:
        """Sets arm to automatically controlled position and orientation mode, 
        """
        self._control_mode = ControlMode.AUTOMATIC
    
    def gripper_mode(self) -> None:
        """Sets arm to gripper mode, 
        """
        self._control_mode = ControlMode.GRIPPER

    def finished(self) -> None:
        """Finishes the control

        Controller will halt sending messages, this will cause the control to leave OFFBOARD.

        See basic_states land for optimal landing procedures

        """
        self._control_mode = ControlMode.FINISHED
        
    def exit(self) -> None:
        """Set the control mode to EXIT, used by control_stack to terminate controller

        Used to cleanly terminate TargetPublisher thread, currently does not stop controller from being set into another mode.

        """
        self._control_mode = ControlMode.EXIT



class ControllerInfoPublisher(ThreadedPublisher):
    """Parallelize controller info publishing
    """

    def __init__(self, controller: Controller, rate=20) -> None:
        super().__init__(rate)
        self._controller = controller
        self._pub = rospy.Publisher(topics.CONTROLLER_INFO, ControllerInfo, queue_size=10)

    def populate_Pos_Rot(self, data_list) -> PosRot:
        """Populates a ROS message with data from a list.

        Args:
            msg: The ROS message to populate.
            data_list: A list of 7 floats representing [x, y, z, qx, qy, qz, qw].
        """
        msg = PosRot()
        msg.pos_x = data_list[0]
        msg.pos_y = data_list[1]
        msg.pos_z = data_list[2]
        msg.rot_x = data_list[3]
        msg.rot_y = data_list[4]
        msg.rot_z = data_list[5]
        msg.rot_w = data_list[6]
        return msg

    def populate_Joint_State(self, data_list) -> UnityJointState:
        msg = UnityJointState()
        msg.joint_1 = data_list[0]
        msg.joint_2 = data_list[1]
        msg.joint_3 = data_list[2]
        msg.joint_4 = data_list[3]
        msg.joint_5 = data_list[4]
        msg.joint_6 = data_list[5]
        msg.joint_7 = data_list[6]
        return msg
        
    def publish(self) -> None:
        controller_info = ControllerInfo()
 
        controller_info.controller_string_info = self._controller.get_controller_info()  # TODO remove when webview in a good state

        controller_info.cc_pose_cur = self.populate_Pos_Rot(self._controller.cc_pose_cur)
        controller_info.cc_pose_init = self.populate_Pos_Rot(self._controller.cc_pose_init)
        controller_info.ee_pose_cur = self.populate_Pos_Rot(self._controller.ee_pose_cur)
        controller_info.ee_pose_target = self.populate_Pos_Rot(self._controller.ee_pose_target)
        controller_info.ee_pose_default = self.populate_Pos_Rot(self._controller.ee_pose_default)

        controller_info.joint_state = self.populate_Joint_State(self._controller.joint_angles)
        controller_info.gripper_state = self.populate_Joint_State(self._controller.gripper_angles + (0,0,0,0,0))
        controller_info.rg_pose_target = self._controller.rg_pose_target

        controller_info.bounding_box_params = self._controller.bounding_box
        controller_info.position_tolerance = self._controller.position_tolerance
        controller_info.orientation_tolerance = self._controller.orientation_tolerance

        controller_info.control_mode = self._controller._control_mode.name
        controller_info.interface_mode = self._controller._interface_mode.mode

        self._pub.publish(controller_info)
        # print("published")

    def exit(self) -> bool:
        return self._controller._control_mode == ControlMode.EXIT

class TargetPublisher(ThreadedPublisher):
    """Parallelize target publishing
    """

    def __init__(self, controller: Controller, rate=20) -> None:
        super().__init__(rate)
        self._controller = controller
        self.gripper_sent = False
    def publish(self) -> None:
        if self._controller.control_mode is ControlMode.GRIPPER:
            if not self.gripper_sent:
                self.gripper_sent = True
                print("Gripper Target Sent: "+ str(self._controller.rg_pose_target))
        else: 
            self._controller.publish_ee_pose()
            self.gripper_sent = False
        # self._controller.publish_rg_pose()
    def exit(self) -> bool:
        return self._controller._control_mode == ControlMode.EXIT
       
