#!/venv/bin/env python3


# Main class
from rclpy.node import Node

# Numpy lib
import numpy as np

# Contants and special functions
from math import pi
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline





class MathUtils(Node):
    
    """
    Math functions class.
    """

    def __init__(self, node_name: str = "math_utils") -> None:
        super().__init__(node_name)
        
        # Cubic interpolation for max linear speed
        self.distance_errors = np.array([0.3, 0.5, 1.0, 2.0, 3.0])
        self.max_speed_points = np.array([0.15, 0.2, 0.3, 0.6, 0.8])
        self.max_speed_function =CubicSpline(self.distance_errors, self.max_speed_points)


    # **********************************************************************************************************
    #                                           Math fuctions                                         
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
        Converts a vector expressed in the NED (North-East-Down) coordinate frame into its equivalent in the body-fixed frame.

        Args:
        vector_body: np.array([x, y, z])

        Returns:
        np.array([N, E, D])
        """
        return np.dot(self.rotation_matrix, vector_body) 
    

    ############################################################## 
    # Convert a NED frame vector to body-fixed frame vector
    ##############################################################

    def convert_ned_to_body(self, vector_ned: np.array) -> np.array:
        """
        Converts a vector expressed in the body-fixed frame into its equivalent in the NED (North-East-Down) coordinate frame.

        Args:
        vector_ned: np.array([N, E, D])

        Returns:
        np.array([x, y, z])
        """
        return np.dot(self.rotation_matrix.T, vector_ned) 
    

    ############################################################## 
    # Convert a body-fixed frame vector to NED frame vector
    ##############################################################

    def compute_velocity_NE(self, target_position: np.array, current_position: np.array, speed: float) -> np.array:
        """
        Calculates the velocity decomposition in the North (N) and East (E) axes in the NED system.

        Args:
        target_pos: np.array([N, E, D]) - Target position (North, East, Downhill)
        current_pos: np.array([N, E, D]) - Current position (North, East, Downhill)
        speed: float - Desired scalar velocity in m/s (default: 0.3 m/s)

        Returns:
        (v_N, v_E): tuple - Velocity components in the N and E axes
        """

        direction = target_position - current_position  # Directional vector
        norm = np.linalg.norm(direction)  # Directional vector norm 
        
        if norm == 0:  # Special case: The drone is already in target position
            return np.array([0.0, 0.0, 0.0])

        unit_direction = direction / norm  # Directional vector normalized (unit vector) 
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
        

    ############################################################## 
    # Get max speed
    ##############################################################

    def get_max_speed(self, err_0: float) -> float:
        """
        Calculates the max speed for a given distance to be percurred 
        Args:
            err_0: Target distance - current distance
        Returns:
            max speed
        """
        return self.max_speed_function(err_0)
    

    ############################################################## 
    # S-curve profile
    ##############################################################

    def s_curve_profile(self, err_0: float, current_error) -> float:
        """
        S-curve profile approximation
        Args:
            err_0: Total distance to be covered
            current_error: Distance that still remains to be covered
        """
        
        # Linear acceleration
        if (current_error > 0.7*err_0) and (current_error <= err_0 + 0.1):      
            return -(1/0.3*err_0)*current_error + 10/3
        
        # Constante velocity, aceleration = 0
        elif (current_error <= 0.7*err_0) and (current_error > 0.55*err_0):
            return 1
        
        # Smoothed deacceleration
        elif (current_error <= 0.55*err_0) and (current_error > 0.15*err_0):
            return self.sigmoid(current_error, 0.08*err_0, 0.55*err_0)
        
        elif (current_error <= 0.15*err_0) and (current_error > 0.05*err_0):
            c = self.sigmoid(0.08*err_0, 0.15*err_0, 0.55*err_0)
            return c * ( (current_error / (0.1*err_0)) - 0.5)
        
        # Error margin
        else:
            return 0
    

    ############################################################## 
    # Sigmoid profile
    ##############################################################

    def sigmoid(self, x, x0, x1):
        """
        Adjusted sigmoid to transition between x0 and x1
        Args:
            x:  point to be calculated
            x0: starting point of the sigmoid
            x1: ending point of the sigmoid
        Returns:
            result: sigmoid calculated in x
        """

        x_mid = (x0 + x1) / 2    # Transition center
        k = 10 / (x1 - x0)       # Slope
        result = 1 / (1 + np.exp( -k * (x - x_mid) ))

        return result

