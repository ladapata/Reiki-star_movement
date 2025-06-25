#!/venv/bin/env python3


# Auxiliary Classes
from movement_controller.math_utils import MathUtils
from movement_controller.px4_controller import Px4Controller 

# Quality of Service policies
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus

# Numpy lib
import numpy as np





class MovementController(Px4Controller, MathUtils):

    """
    Node for controlling a vehicle in offboard mode.
    """

    def __init__(self, node_name: str = "movement_controller") -> None:
        super().__init__(node_name)       
    

    # **********************************************************************************************************
    #                                            Movement methods
    # **********************************************************************************************************


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

    def move(self, x: float = 0.0, y: float = 0.0) -> bool:

        """
        Moves the vehicle to a relative position (x,y) at specified speed.
        Returns True when movement is completed, False otherwise.
        Works only in OFFBOARD mode when armed.
        """
        
        # Check if this is the first execution
        if not hasattr(self, 'move_initialized'):
            self.move_initialized = True  # Marks that initialization was done

            # calculates how much to move in NED coordinates
            self.distance_ned = self.convert_body_to_ned(np.array([x, y, 0])) # Don't move in z axis

            # target position is current position plus the distance to move (distance_ned)
            self.target_position_ned = np.array([
                self.vehicle_local_position.x + self.distance_ned[0],
                self.vehicle_local_position.y + self.distance_ned[1],
                self.vehicle_local_position.z
            ])

            # distance error vector
            error_vector = np.array([
                self.vehicle_local_position.x - self.target_position_ned[0],
                self.vehicle_local_position.y - self.target_position_ned[1],
                0
            ])

            # norm of distance error vector
            self.err_0 = np.linalg.norm(error_vector)

            if self.err_0 > 3:
                self.max_speed = 0.8
            else:
                self.max_speed = self.get_max_speed(self.err_0)
    
        # Current distance to target
        current_error = np.linalg.norm(
                np.array([
                    abs(self.vehicle_local_position.x - self.target_position_ned[0]),
                    abs(self.vehicle_local_position.y - self.target_position_ned[1]),
                    0
                ])
            )

        # Calculate smoothed speed
        speed_factor = self.s_curve_profile(self.err_0, current_error)
        speed = self.max_speed * speed_factor

        self.get_logger().info(f"factor = {speed_factor}")
        self.get_logger().info(f"speed = {speed}")


        # computes NED velocity components in the movement direction
        self.velocity_ned = self.compute_velocity_NE(
            self.target_position_ned, 
            np.array([
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z,]), 
            speed
        )

        # Check if vehicle is in OFFBOARD mode and ARMED
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD \
                and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            

            # Publish velocity command
            self.publish_trajectory_setpoint(vx=self.velocity_ned[0], vy=self.velocity_ned[1])

            if round(self.velocity_ned[0], 2) == 0 and round(self.velocity_ned[1], 2) == 0:
                #self.get_logger().info('Movimento concluído.')

                # Delete attributes to reuse the function
                delattr(self, 'move_initialized')
                delattr(self, 'distance_ned')
                delattr(self, 'target_position_ned')
                delattr(self, 'velocity_ned')
                delattr(self, 'err_0')
                delattr(self, 'max_speed')
                return True  # Movement completed
            else:
                return False  # Movement not finished yet
            
        return False  # Not in OFFBOARD mode or ARMED


    ############################################################## 
    # Turn around z-axis (yaw)
    ##############################################################

    def turn(self, angle: float = 0.0, orientation: str = 'right', yaw_tolerance: float = 0.0174, yaw_rate: float = 0.13) -> bool:
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

            # Calculate how much to move
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
            if yaw_error > yaw_tolerance*4:
                speed = 2 * yaw_rate
            elif yaw_error > yaw_tolerance*3:
                speed = 1.25 * yaw_rate
            elif yaw_error > yaw_tolerance*2:
                speed = 0.75* yaw_rate
            elif yaw_error > yaw_tolerance:
                speed = 0.25* yaw_rate
            else:
                speed = 0.0

            # speed = exp(yaw_error - 3) * yaw_rate
            #self.get_logger().info(f"speed = {speed}")

            if speed < 0.03: speed = 0

            # Define the orientation
            orientation = 1 if (orientation == 'right') else (-1)

            # Send rotation msg
            self.publish_trajectory_setpoint(yaw_rate=float(orientation*speed))

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