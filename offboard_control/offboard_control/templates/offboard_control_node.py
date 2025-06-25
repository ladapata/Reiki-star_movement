#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, \
    VehicleLocalPosition, VehicleStatus, VehicleAttitude, VehicleLandDetected

import numpy as np
from math import pi, nan
from scipy.spatial.transform import Rotation as R


class OffboardControlNode(Node):

    """
    Node for controlling a vehicle in offboard mode.
    """

    def __init__(self, node_name: str = "offboard_control") -> None:
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

        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_land_detected = VehicleLandDetected()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.rotation_matrix = None
    
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


    # **********************************************************************************************************
    #                                            Movement methods
    # **********************************************************************************************************


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


    ############################################################## 
    # Takeoff the drone
    ##############################################################

    def takeoff(self, takeoff_height: float, speed: float = 0.5) -> bool:
        """
        Moves the vehicle to a relative position (z) at specified speed.
        Returns True when movement is completed, False otherwise.
        Works only in OFFBOARD mode when armed.
        """

        # Move the drone after in offboard and armed mode
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD \
            and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:

                # Publish speed in z-axis until the height has been reached
                if abs(self.vehicle_local_position.z) < takeoff_height:

                    # Land when reach the target height
                    self.publish_trajectory_setpoint(vz=speed)
                    return False
                else:
                    self.publish_trajectory_setpoint(vz=0.0)
                    return True
        return False
        

    ############################################################## 
    # Move the drone in xy plan
    ##############################################################

    def move(self, x: float = 0.0, y: float = 0.0, speed: float = 0.4) -> bool:
        """
        Moves the vehicle to a relative position (x,y) at specified speed.
        Returns True when movement is completed, False otherwise.
        Works only in OFFBOARD mode when armed.
        """
        
        # Check if this is the first execution
        if not hasattr(self, 'move_initialized'):
            self.move_initialized = True  # Marks that initialization was done

            # calculates how much to move in NED coordinates
            self.distance_ned = self.convert_body_to_ned(np.array([x, y, 0]))

            # target position is current position plus the distance to move (distance_ned)
            self.target_position_ned = np.array([
                self.vehicle_local_position.x + self.distance_ned[0],
                self.vehicle_local_position.y + self.distance_ned[1],
                self.vehicle_local_position.z
            ])

            # computes NED velocity components in the movement direction
            self.velocity_ned = self.compute_velocity_NE(self.target_position_ned, np.array([
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z,
            ]), speed)

        # Check if vehicle is in OFFBOARD mode and ARMED
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD \
                and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:

            # Smoth x velocity
            if abs(self.vehicle_local_position.x - self.target_position_ned[0]) > 0.10:
                velocity_x = round(float(self.velocity_ned[0]), 1)
            elif abs(self.vehicle_local_position.x - self.target_position_ned[0]) > 0.05:
                velocity_x = 0.1
            else:
                velocity_x = 0.0

            # Smoth y velocity
            if abs(self.vehicle_local_position.y - self.target_position_ned[1]) > 0.10:
                velocity_y = round(float(self.velocity_ned[1]), 1)
            elif abs(self.vehicle_local_position.y - self.target_position_ned[1]) > 0.05:
                velocity_y = 0.1
            else:
                velocity_y = 0.0

            # Publish velocity command
            self.publish_trajectory_setpoint(vx=velocity_x, vy=velocity_y)

            if velocity_x!=0.0:
                return False  # Movement not finished yet
            elif velocity_y!=0.0:
                return False  # Movement not finished yet
            else:
                #self.get_logger().info('Movimento concluído.')
                # Delete attributes to reuse the function
                delattr(self, 'move_initialized')
                delattr(self, 'distance_ned')
                delattr(self, 'target_position_ned')
                delattr(self, 'velocity_ned')
                return True  # Movement completed
            
        return False  # Not in OFFBOARD mode or ARMED


    ############################################################## 
    # Turn around z-axis (yaw)
    ##############################################################

    def turn(self, angle: float = 0.0, orientation: str = 'right', yaw_tolerance: float = 0.174, yaw_rate: float = 0.5) -> bool:
        """
        Performs rotation around the z axis in a smoothed manner.
        Returns True when movement is completed, False otherwise.
        Works only in OFFBOARD mode when armed.
        """

        # Convert angle from degrees to radians
        angle_rad = np.radians(angle)

        # Check if is the first time running
        if not hasattr(self, 'turn_initialized'):
            self.turn_initialized = True  # Marks that initialization was done

            # Calculate how much to move in NED coordinates
            self.target_yaw = self.get_target_yaw(
                angular_distance=angle_rad, 
                current_yaw=self.yaw, 
                orientation=orientation
            )

        # Check if vehicle is in OFFBOARD mode and ARMED
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD \
            and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:

            # Calcule the yaw error
            yaw_error = self.get_yaw_error(
                current_yaw=self.yaw, 
                target_yaw=self.target_yaw
            )

            # Regulates the rotation speed
            if yaw_error > yaw_tolerance*3:
                speed = 2 * yaw_rate
            elif yaw_error > yaw_tolerance*2:
                speed = 1.25 * yaw_rate
            elif yaw_error > yaw_tolerance:
                speed = 0.75* yaw_rate
            elif yaw_error > yaw_tolerance/2:
                speed = 0.25* yaw_rate
            else:
                speed = 0.0
                
            orientation = 1 if (orientation == 'right') else (-1)

            self.publish_trajectory_setpoint(yaw_rate=orientation*speed)

            if speed != 0:
                return False  # Movement not finished yet
            else:
                # Movement completed
                self.get_logger().info('Movement completed.')
                # Delete attributes to reuse the function
                if hasattr(self, 'turn_initialized'):
                    delattr(self, 'turn_initialized')
                
                if hasattr(self, 'target_yaw'):
                    delattr(self, 'target_yaw')

                return True  # Movement completed
        return False  # Not in OFFBOARD mode or ARMED


    # **********************************************************************************************************
    #                                            Auxiliary methods
    # **********************************************************************************************************
    

    ############################################################## 
    # Get rotation matriz (From NED frame to Body-Fixed)
    ##############################################################

    def quaternion_to_rotation_matrix(self, q):
        """
        Convert a quaternion q(w,x,y,z) to a rotation matriz.
        The quaternions come from controler running PX4.
        """
        r = R.from_quat([q[1], q[2], q[3], q[0]])  # scipy uses q(x, y, z, w)
        return r.as_matrix()
    

    ############################################################## 
    # Get the yaw angle from quaternions
    ##############################################################

    def get_yaw(self, q_w, q_x, q_y, q_z):
        """
        Calculates yaw based on the orthogonal projection of quaternions onto the xy plane avoiding ambiguity.
        """
        return np.arctan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))


    ############################################################## 
    # Convert a body-fixed frame vector to NED frame vector
    ##############################################################

    def convert_body_to_ned(self, vector_body: np.array) -> np.array:
        """
        Transforms a vector in coordinates in the fixed frame system in the body to a vector in NED coordinates.

        Args:
        vector_body: np.array([N, E, D])

        Returns:
        np.array([local_x, local_y, local_z])
        """

        
        return np.dot(self.rotation_matrix, vector_body) 
    

    ############################################################## 
    # Convert a body-fixed frame vector to NED frame vector
    ##############################################################

    def compute_velocity_NE(self, target_position: np.array, current_position: np.array, speed: float = 0.2) -> np.array:
        """
        Calculates the velocity decomposition in the North (N) and East (E) axes in the NED system.

        Args:
        target_pos: np.array([N, E, D]) - Target position (North, East, Downhill)
        current_pos: np.array([N, E, D]) - Current position (North, East, Downhill)
        speed: float - Desired scalar velocity in m/s (default: 0.3 m/s)

        Returns:
        (v_N, v_E): tuple - Velocity components in the N and E axes
        """

        direction = target_position - current_position  # Directinal vector
        norm = np.linalg.norm(direction)  # Directinal vector norm 
        
        if norm == 0:  # Special case: The drone is already in target position
            return np.array(0.0, 0.0, 0.0)

        unit_direction = direction / norm  # Directinal vector normalized (unit vector) 
        velocity_vector = speed * unit_direction  # Multiply by speed

        return velocity_vector
    

    ############################################################## 
    # Convert a body-fixed frame vector to NED frame vector
    ##############################################################

    def get_target_yaw(self, angular_distance: float, current_yaw: float, orientation: str = 'right') -> float:
        """
        Calculates the target yaw angle after rotation, handling angle wrapping between -π and +π.
        Args:
            angular_distance: The angle to rotate (radians)
            current_yaw: Current yaw angle (radians)
            orientation: Rotation direction ('right' or 'left')
        Returns:
            Target yaw angle within [-π, π] range
        """
        
        if current_yaw > 0:                                                  # if current yaw is positive
            if orientation == 'right':                                       # and wants to move right
                if current_yaw + angular_distance < 0.0:                     # if crossed (-π, +π) boundary
                    return -(pi-(current_yaw - angular_distance - pi))       # adjust angle to valid range 
                else:                                                        # if within boundary limits
                    return current_yaw + angular_distance                    # return simple sum
            elif orientation == 'left':                                      # if wants to move left
                return current_yaw - angular_distance                        # return simple difference
        else:                                                                # if current yaw is negative
            if orientation == 'right':                                       # and wants to move right
                return current_yaw + angular_distance                        # return simple sum
            elif orientation == 'left':                                      # if wants to move left
                if current_yaw - angular_distance > 0:                       # if final angle exceeds boundary
                    return pi + ((current_yaw - angular_distance) + pi)      # adjust to valid range
                else:                                                        # if difference is within range
                    return current_yaw - angular_distance                    # return simple difference


    ############################################################## 
    # Calculate yaw error
    ##############################################################

    def get_yaw_error(self, current_yaw: float, target_yaw: float) -> float:
        """
        Calculates the target yaw error, handling angle wrapping between -π and +π.
        Args:
            current_yaw: Current yaw angle (radians)
            target_yaw: Target yaw angle (radians)
        Returns:
            Yaw error angle within [-π, π] range
        """
        if target_yaw - current_yaw > 0:
            return abs( (target_yaw - current_yaw + pi) % (2*pi) - pi )
        else:
            return abs( (target_yaw - current_yaw + pi + (2*pi)) % (2*pi) - pi )
