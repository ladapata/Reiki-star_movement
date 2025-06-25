#!/usr/bin/env python3

import rclpy
from enum import Enum, auto
from offboard_control.templates.offboard_control_node import OffboardControlNode
from px4_msgs.msg import VehicleStatus


# Difine the state of state machine
class State(Enum):
    IDLE = auto()           # Initial State
    TAKEOFF = auto()        # Takeoff
    MOVE = auto()           # Move
    TURN = auto()           # Turn
    LAND = auto()           # Land
    DISARM = auto()         # Disarm
    FINISHED = auto()       # Task finished

class StateMachine(OffboardControlNode):
    """
    State machine class to implement some logic.    # MODIFY: Class description.
    """

    ############################################################## 
    # Constructor
    ##############################################################

    def __init__(self):
        super().__init__('square_movement')  # MODIFY: Nome name.

        # Initial State
        self.state = State.IDLE

        self.timer_rate = 10.0       # [Hz] Must be greater than 2Hz. 

        # Timer to call state machine loop method
        self.timer = self.create_timer(1/self.timer_rate, self.state_machine_loop) 

        # Waiting time before arm the drone
        self.waiting_time = 20 

        # Count how many times state_machine_loop was called by the timer
        self.offboard_setpoint_counter = 0

        # Atributes to control actions
        self.takeoff_complete = False
        self.move_complete = False
        self.turn_complete = False

    ############################################################## 
    # State machine loop running at
    ##############################################################

    def state_machine_loop(self):
        """Main function of the state machine."""
        if self.state == State.IDLE:
            self.idle_state()
        elif self.state == State.TAKEOFF:
            self.takeoff_state()
        elif self.state == State.MOVE:
            self.move_state()
        elif self.state == State.TURN:
            self.turn_state()
        elif self.state == State.LAND:
            self.land_state()
        elif self.state == State.DISARM:
            self.disarm_state()
        elif self.state == State.FINISHED:
            self.finished_state()


    ############################################################## 
    # IDLE State
    ##############################################################

    def idle_state(self):
        """IDLE State logic."""
        #self.get_logger().info('State: IDLE')     # OPTIONAL DEBUG

        # Just send heartbeat signal while waiting time hasn't passed
        self.publish_offboard_control_heartbeat_signal()

        # Wait for a few cycles before switching to offboard mode
        if self.offboard_setpoint_counter < self.waiting_time:  # Wait a few seconds before start
            self.offboard_setpoint_counter += 1
            return

        # When the waiting time has passed, switch to engage_offboard_mode and then arm the drone
        if self.offboard_setpoint_counter == self.waiting_time:

            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.arm()
                return
            else:
                if not self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.engage_offboard_mode()
                    return 
            
        # IDLE state exit condition
        if self.takeoff_complete == False:
            self.state = State.TAKEOFF  # Transition to TAKEOFF
            #self.get_logger().info('Transition to TAKEOFF')


    ############################################################## 
    # Takeoff state
    ##############################################################

    def takeoff_state(self):
        """TAKEOFF State logic."""
        #self.get_logger().info('State: TAKEOFF')     # OPTIONAL DEBUG

        # Send signal to keep conection
        self.header()

        self.takeoff_complete = self.takeoff(takeoff_height=2.0)

        # When takeoff is finished, goes to next state
        if self.takeoff_complete:
            self.state = State.MOVE  # Transition to MOVE
            #self.get_logger().info('Transition to MOVE')     # OPTIONAL DEBUG

        return


    ############################################################## 
    # Action 1 State           MODIFY
    ##############################################################

    def move_state(self):
        """MOVE State logic."""
        #self.get_logger().info('State: TURN')
        
        self.header()
        self.move_complete = self.move(x=2.0) 

        if self.move_complete:
            self.state = State.TURN # Transition to TURN
            #self.get_logger().info('Transition to TURN')     # OPTIONAL DEBUG

    
    ############################################################## 
    # Action 2 State           MODIFY
    ##############################################################

    def turn_state(self):
        """TURN State logic."""
        #self.get_logger().info('State: TURN')
        
        self.header()
        self.turn_complete = self.turn(angle=90.0, orientation='right') 

        if self.turn_complete:
            self.state = State.MOVE  # Transition to MOVE
            #self.get_logger().info('Transition to MOVE')     # OPTIONAL DEBUG


    ############################################################## 
    # Land State         
    ##############################################################

    def land_state(self):
        """LAND State logic."""
        #self.get_logger().info('State: LAND')
        self.publish_offboard_control_heartbeat_signal()

        if not self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.land()
            return

        if self.vehicle_land_detected.landed:
            self.state = State.DISARM  # Transition to DISARM
            #self.get_logger().info('Transition to DISARM')     # OPTIONAL DEBUG


    ##############################################################
    # Disarm State          
    ##############################################################

    def disarm_state(self):
        """DISARM State logic."""
        #self.get_logger().info('State: DISARM')
        self.publish_offboard_control_heartbeat_signal()
        
        self.disarm()
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.state = State.FINISHED
            #self.get_logger().info('Transition to FINISHED')


    ##############################################################
    # Finished State          
    ##############################################################

    def finished_state(self):
        """FINISHED State logic."""
        #self.get_logger().info('State: FINISHED')

        self.get_logger().info('Task completed. Closing...')
        exit(0)

    ##############################################################
    # Header to keep conextion          
    ##############################################################

    def header(self):
        self.publish_offboard_control_heartbeat_signal()
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.arm()
                return
        else:
            if not self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.engage_offboard_mode()
                    return    
        

def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachine()
    rclpy.spin(state_machine)  # Mantém o nó ativo
    state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()