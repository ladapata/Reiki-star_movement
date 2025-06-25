#!/usr/bin/env python3

import rclpy
from enum import Enum, auto
from offboard_control.templates.offboard_control_node import OffboardControlNode
from px4_msgs.msg import VehicleStatus


# Difine the state of state machine
class State(Enum):
    IDLE = auto()           # Initial State
    TAKEOFF = auto()        # Takeoff
    WALK = auto()           # Permorm action 1
    TURN = auto()           # Permorm action 2 [etc...]
    LAND = auto()           # Land
    USER_INPUT = auto()
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
        super().__init__('node_name')  # MODIFY: Nome name.

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
        self.walk_complete = False
        self.turn_complete = False

        self.points = 0
        self.angle = 0
        self.star_done = 0

    ############################################################## 
    # State machine loop running at
    ##############################################################

    def state_machine_loop(self):
        """Main function of the state machine."""
        if self.state == State.IDLE:
            self.idle_state()
        elif self.state == State.TAKEOFF:
            self.takeoff_state()
        elif self.state == State.USER_INPUT:
            self.input_state()
        elif self.state == State.WALK:
            self.walk_state()
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

        self.takeoff_complete = self.takeoff(takeoff_height=1.0)

        # When takeoff is finished, goes to next state
        if self.takeoff_complete:
            self.state = State.USER_INPUT
            #self.get_logger().info('Transition to TURN')     # OPTIONAL DEBUG

        return


    def input_state(self):
    	self.header()
    	self.points = 5
    	self.angle = 180.0 - (180.0 / self.points)
    	self.state = State.WALK


    def walk_state(self):
        self.header()
        self.walk_complete = self.move(x=1.0)

        if self.walk_complete:
            self.star_done += 1
            if self.star_done == self.points:
                self.state = State.LAND
            else:
                self.state = State.TURN


    def turn_state(self):
        self.header()
        self.turn_complete = self.turn(self.angle, orientation='left')

        if self.turn_complete:
            self.state = State.WALK


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
