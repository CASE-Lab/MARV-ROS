#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV Thrust Test Scenario             #
# By Viktor Lindstrom, Spring 2021      #
# Chalmers University of Technology     #
# ------------------------------------- #
# This scenario increases the APS output in several steps. Together with
# RPM and Velocity measurements it should be possible to estimate the thrust function.
'''

# System imports
import time

# Custom libraries
from timer import Timer
from scenario import Scenario

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

# Messages
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import Float32
from std_msgs.msg import Int32

class Thrust_Test(Scenario):

    ##################################### INIT #############################################################

    def __init__(self):
        '''The Init function is run on node start. Should not subscribe, publish or start any timers
            except for the "run_setup_check_timer". This is because we don't want anything running in the 
            background when this scenario is not running.
        '''
        # Scenario name parameter (this needs to be added/same as the name in /marv_driver/config/scenarios.yaml)
        __scenario_name = "THR_TST"
        # Scenario data variable headers, set to "" if not used, max 7 characters
        __data_var_1_header = "RPM"
        __data_var_2_header = "CUR_VEL"

        # Initialize Scenario Wrapper (handles the logic for starting and stopping scenarios)
        super().__init__('thrust_test',__scenario_name,__data_var_1_header,__data_var_2_header)

        self.get_logger().info("Initializing Scenario Node: " + __scenario_name)

        # Configuration parameters 
        self.main_loop_update_period = 0.01 # seconds update time for the main loop, where the controller should be

        # ------- Global variables -------
        self.run_setup = True # Whenever to run the setup method when in INITIALIZING mode

        self.node_start_delay_timer = Timer()
        self.aps_signal = 0.0
        self.aps_signal_steps = [5.0,15.0,25.0,35.0,45.0,55.0,65.0] 
        self.aps_signal_steps_index = 0
        self.aps_signal_max = 65.0 # Cap aps value for safety
        self.current_rpm = 0
        self.current_velocity = 0
        
        # Timer for keeping the time for each aps signal step, when to switch to the next
        self.steps_timer = Timer()
        self.steps_timer.stop()
        self.steps_time = 10.0 # Run each step for
        # --------------------------------

        # Check setup timer config
        self.__run_setup_check_timer_period = 0.1 # seconds between checking if the setup should be run (which should be done when the scenario is running)

        # Check setup timer
        self.__run_setup_check_timer = self.create_timer(self.__run_setup_check_timer_period,self.__run_setup_check_callback)

        self.get_logger().info("Started Scenario Node: " + __scenario_name)

    ########################################################################################################

    ##################################### SETUP, TEARDOWN AND CHECK ########################################

    def __run_setup_check_callback(self):
        # If scenario is being initialized, run setup
        if self.run_setup and self.get_scenario_state() == "INITIALIZING":
            self.run_setup = False # Prevent the setup from being run several times
            self.__setup() # Call setup
        elif not self.run_setup and self.get_scenario_state() == "STOPPED":
            self.__teardown()
            self.run_setup = True

    def __setup(self):
        '''Setup configuration parameters, global variables, subscriptions, publishes and timers.
            This function is run when the scenario is initializing, i.e. requested to start but not 
            yet runnung (we are still in manual mode which needs to be changed to external to start)
        '''

        # Set variables
        self.aps_signal = 0.0
        self.aps_signal_steps_index = 0
        self.steps_timer.stop()

        # Subscriptions
        self.velocity_subscription = self.create_subscription(Float32, '/marv/nav/sbg_velocity_magnitude', self.marv_velocity_magnitude_callback, 10)
        self.velocity_subscription
        self.rpm_subscription = self.create_subscription(Int32, '/marv/sys/status/motor_rpm', self.marv_rpm_measurement_callback, 10)
        self.rpm_subscription

        # Publishes

        # Allow time for subscriptions and publishes to be set up
        time.sleep(0.5)

        # Custom timers
        self.node_start_delay_timer.stop()

        # Timers
        # Set a timer for the main_loop_callback (where the scenario is running)
        self.main_loop_timer = self.create_timer(self.main_loop_update_period, self.main_loop_callback)

    def __teardown(self):
        '''Teardown timers, publishers and subscriptions when the scenario is no longer running'''
        
        self.destroy_subscription(self.velocity_subscription)
        self.destroy_subscription(self.rpm_subscription)
        #self.destroy_publisher(self.__node_state_publisher_)

        self.main_loop_timer.cancel()

    ########################################################################################################

    ##################################### MAIN LOOP CALLBACK ###############################################

    # Main program loop
    def main_loop_callback(self):
        '''When the external mode is activated, get_scenario_running() will turn true. After that
            the steering commands needs to be updated with at least 10 Hz, using self.update_cmd_steering(), otherwise there will be
            a system error since the PDU will sense that the updates has stopped.
        '''

        if self.get_scenario_running():
            # Wait 3 seconds before starting
            if self.node_start_delay_timer.elapsed() == 0:
                self.node_start_delay_timer.start()

            # After 3 seconds start controller
            elif self.node_start_delay_timer.elapsed() > 3:
                if self.steps_timer.elapsed() == 0:
                    self.steps_timer.start()
                elif self.steps_timer.elapsed() >= self.steps_time:
                    self.steps_timer.reset()
                    
                    if self.aps_signal_steps_index < len(self.aps_signal_steps)-1:
                        self.aps_signal_steps_index += 1
                    else:
                        self.set_state_finished()

                self.aps_signal = self.aps_signal_steps[self.aps_signal_steps_index]
                if self.aps_signal > self.aps_signal_max:
                    self.aps_signal = self.aps_signal_max

                self.set_cmd_steering_aps(self.aps_signal)

            self.set_progress(str(round(self.aps_signal,2)) + "|" + str(int(self.aps_signal_steps_index)) + "/" + str(int(len(self.aps_signal_steps)))) # Show The time that we are within the goal velocity
            self.set_data_var_1(str(round(self.current_rpm,2))) # Show RPM
            self.set_data_var_2(str(round(self.current_velocity,2)) + "/" + str(self.current_velocity)) # Show velocity

            self.update_cmd_steering() # Send the complete steering command to the system
            self.update_all_variables() # Send the updated variables to the system

            self.get_logger().info("APS: " + str(round(self.aps_signal,2)))

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    # Get current velocity from INS
    def marv_velocity_magnitude_callback(self,ros_msg):
        self.current_velocity = ros_msg.data

    # Get current RPM
    def marv_rpm_measurement_callback(self,ros_msg):
        self.current_rpm = ros_msg.data

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    thrust_test = Thrust_Test()

    rclpy.spin(thrust_test)

    thrust_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
