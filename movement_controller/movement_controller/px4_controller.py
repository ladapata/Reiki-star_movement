#!/venv/bin/env python3


# Main class
from rclpy.node import Node

# Quality of Service policies
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Message interfaces used by publishers and subscribers
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, \
    VehicleLocalPosition, VehicleStatus, VehicleAttitude, VehicleLandDetected

# NaN (not a number) from math
from math import nan





class Px4Controller(Node):

    """
    Node for controlling a px4 based vehicle in offboard mode.
    """

    def __init__(self, node_name: str = "px4_controller") -> None:
        super().__init__(node_name)

        ##############################################################
        # Configure QoS profile for publishing and subscribing
        ##############################################################

        # quality of service
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        ############################################################## 
        # Create publishers
        ##############################################################

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        ############################################################## 
        # Create subscribers
        ##############################################################

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback,
            qos_profile
        )

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile
        )

        self.vehicle_land_detected_subscriber = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.vehicle_land_detected_callback,
            qos_profile
        )

        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile
        )

        ############################################################## 
        # Declare some usefull parameters
        ##############################################################

        # Messages coming from pixhawk
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_land_detected = VehicleLandDetected()

        # Geolocalization matrix and yaw
        self.rotation_matrix = None
        self.yaw = 0.0

    
    ############################################################## 
    # Publishers callback
    ##############################################################

    def vehicle_local_position_callback(self, vehicle_local_position):
        """
        Callback function for vehicle_local_position topic subscriber.
        """
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """
        Callback function for vehicle_status topic subscriber.
        """
        self.vehicle_status = vehicle_status

    def vehicle_land_detected_callback(self, vehicle_land_detected):
        """
        Callback function for vehicle_land_detected topic subscriber.
        """
        self.vehicle_land_detected = vehicle_land_detected

    def vehicle_attitude_callback(self, vehicle_attitude):
        """
        Callback function for vehicle_attitude topic subscriber.
        """

        # Get quartenions from received message
        q = [
            vehicle_attitude.q[0], 
            vehicle_attitude.q[1], 
            vehicle_attitude.q[2], 
            vehicle_attitude.q[3]
        ]
        
        # get rotation matrix
        self.rotation_matrix = self.quaternion_to_rotation_matrix(q)

        # get yaw
        self.yaw = self.get_yaw(q[0], q[1], q[2], q[3])


    # **********************************************************************************************************
    #                                           Basic MAVLink actions                                         
    # **********************************************************************************************************


    ############################################################## 
    # Publish Vehicle Commands
    ##############################################################

    def publish_vehicle_command(self, command, **params) -> None:
        """
        Publish a vehicle command. All commands (such as arm, disarm, switch mode...) are published over this function.
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


    ############################################################## 
    # Arm the drone
    ##############################################################

    def arm(self):
        """
        Send an arm command to the vehicle.
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=1.0
        )
        #self.get_logger().info('Arm command sent')


    ############################################################## 
    # Disarm the drone
    ##############################################################

    def disarm(self):
        """
        Send a disarm command to the vehicle.
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=0.0
        )
        #self.get_logger().info('Disarm command sent')

    ############################################################## 
    # Engage offboard mode
    ##############################################################

    def engage_offboard_mode(self):
        """
        Switch to offboard mode.
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            param1=1.0, 
            param2=6.0
        )
        #self.get_logger().info("Switching to offboard mode")


    ############################################################## 
    # Land the drone
    ##############################################################

    def land(self):
        """
        Switch to land mode.
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND) # commnad to land
        #self.get_logger().info("Switching to land mode")


    ############################################################## 
    # Send a heartbeat signal to keep conection with PX4 software
    ##############################################################

    def publish_offboard_control_heartbeat_signal(self):
        """
        Publish the offboard control mode.
        """
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_publisher.publish(msg)


    ############################################################## 
    # Move the drone sending velocity comands
    ##############################################################

    def publish_trajectory_setpoint(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0, yaw_rate: float = 0.0) -> None:
        """
        Publish the trajectory setpoint, moving the drone by velocity.
        """
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [nan, nan, nan]  # Define position as NaN (Not a Number)
        msg.velocity = [vx, vy, -vz]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]
        msg.yaw = nan
        msg.yawspeed = yaw_rate
        self.trajectory_setpoint_publisher.publish(msg)
