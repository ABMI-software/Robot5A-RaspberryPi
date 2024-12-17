import rclpy
from rclpy.node import Node
from hardware_interface import SystemInterface
from hardware_interface.types import CommandInterfaceType, StateInterfaceType
from sensor_msgs.msg import JointState
import Slush


class SlushEngineHardware(SystemInterface):
    """
    ROS2 hardware interface for the SlushEngine board, using visual servoing for joint states.
    """

    def __init__(self):
        super().__init__()
        self.board = None
        self.joints = {}
        self.position_commands = {}
        self.position_states = {}
        self.joint_names = ["R0_Yaw", "R1_Pitch", "R2_Pitch", "R3_Yaw", "R4_Pitch"]

        # Subscriber for visual joint states
        self.visual_joint_state_subscriber = None

    def on_init(self):
        """
        Initializes the hardware interface and SlushEngine.
        """
        try:
            self.board = Slush.sBoard()
            self.get_logger().info("SlushEngine board initialized.")
            for i, joint_name in enumerate(self.joint_names):
                motor = Slush.Motor(i)
                self.joints[joint_name] = motor
                self.position_commands[joint_name] = 0.0
                self.position_states[joint_name] = 0.0
        except Exception as e:
            self.get_logger().error(f"Error initializing SlushEngine: {e}")
            raise RuntimeError(f"Failed to initialize SlushEngine: {e}")

    def configure(self):
        """
        Configures the SlushEngine and initializes ROS 2 subscriber.
        """
        try:
            # Configure motors
            for joint_name, motor in self.joints.items():
                motor.setMicroSteps(16)
                motor.setCurrent(50, 50,50,50) #
                self.get_logger().info(f"{joint_name} motor configured.")

            # Initialize ROS 2 subscriber
            self.visual_joint_state_subscriber = self.create_subscription(
                JointState,
                "visual_joint_states",
                self.visual_joint_state_callback,
                10
            )
            self.get_logger().info("Subscribed to visual_joint_states topic.")
            return True
        except Exception as e:
            self.get_logger().error(f"Error configuring hardware: {e}")
            return False

    def visual_joint_state_callback(self, msg):
        """
        Callback function for the visual joint states topic.
        Updates the joint state values based on the message.
        """
        for i, name in enumerate(msg.name):
            if name in self.position_states and i < len(msg.position):
                self.position_states[name] = msg.position[i]

    def export_state_interfaces(self):
        """
        Exports the state interfaces for all joints.
        """
        state_interfaces = []
        for joint_name in self.joint_names:
            state_interfaces.append((joint_name, StateInterfaceType.POSITION))
        return state_interfaces

    def export_command_interfaces(self):
        """
        Exports the command interfaces for all joints.
        """
        command_interfaces = []
        for joint_name in self.joint_names:
            command_interfaces.append((joint_name, CommandInterfaceType.POSITION))
        return command_interfaces

    def read(self):
        """
        Reads joint states from the visual_joint_states topic.
        """
        self.get_logger().debug(f"Read joint states: {self.position_states}")

    def write(self):
        """
        Writes position commands to the motors.
        """
        for joint_name, motor in self.joints.items():
            command = self.position_commands[joint_name]
            motor.goTo(command)
            self.get_logger().debug(f"Sent command {command} to {joint_name}")

    def start(self):
        """
        Starts the hardware interface.
        """
        self.get_logger().info("Starting SlushEngine hardware interface...")
        self.read()
        self.write()

    def stop(self):
        """
        Stops the hardware interface.
        """
        self.get_logger().info("Stopping SlushEngine hardware interface...")
