#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV Test Constant Velocity           #
# By Viktor Lindstrom, Spring 2021      #
# Chalmers University of Technology     #
# ------------------------------------- #
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

class Test_Constant_Velocity(Scenario):

    ##################################### INIT #############################################################

    def __init__(self):
        '''The Init function is run on node start. Should not subscribe, publish or start any timers
            except for the "run_setup_check_timer". This is because we don't want anything running in the 
            background when this scenario is not running.
        '''
        # Scenario name parameter (this needs to be added/same as the name in /marv_driver/config/scenarios.yaml)
        __scenario_name = "CST_VEL"
        # Scenario data variable headers, set to "" if not used, max 7 characters
        __data_var_1_header = "APS"
        __data_var_2_header = "CUR_VEL"

        # Initialize Scenario Wrapper (handles the logic for starting and stopping scenarios)
        super().__init__('test_constant_velocity',__scenario_name,__data_var_1_header,__data_var_2_header)

        self.get_logger().info("Initializing Scenario Node: " + __scenario_name)

        # Configuration parameters 
        self.main_loop_update_period = 0.01 # seconds update time for the main loop, where the controller should be
        self.get_new_params_period = 5.0 # seconds update time for updating with new rosparams

        # Declare rosparams
        self.declare_parameters(
            namespace='',
            parameters=[
                ('throttle_aps.Kp', None),
                ('throttle_aps.Ki', None),
                ('throttle_aps.Kd', None),
            ])

        # ------- Global variables -------
        self.run_setup = True # Whenever to run the setup method when in INITIALIZING mode

        self.node_start_delay_timer = Timer()
        self.current_velocity_updated_timer = Timer()
        self.current_velocity_update_timeout = 1 # second before counting the velocity input data as too outdated, signaling fault
        self.current_velocity = 0.0
        self.reference_velocity = 5 # m/s, the goal
        self.aps_signal = 0.0
        self.aps_signal_max = 40.0 # Cap aps value for safety

        # Variables for PID reg
        self.proportional_term = 0.0 
        self.integral_term = 0.0
        self.derivative_term = 0.0
        self.Kp = self.get_parameter('throttle_aps.Kp').value # 1.0
        self.Ki = self.get_parameter('throttle_aps.Ki').value # 0.01
        self.Kd = self.get_parameter('throttle_aps.Kd').value
        self.error = 0 # Error term for PID reg
        self.error_old = 0 # Old error term for PID reg
        
        # Variables for finished condition (to determine if this scenario has succeeded
        self.finished_timer = Timer()
        self.finished_velocity_intervl = 0.3 # +/- velocity where we count the scenario as finished after a certain time
        self.finished_waiting_time = 10 # seconds to wait before counting the scenario as finished if we are within the velocity interval
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

        # Subscriptions (subscribe to for example sensor data)
        self.pose_subscription = self.create_subscription(PoseWithCovariance, '/marv/nav/sbg_pose', self.marv_pose_callback, 10)
        self.pose_subscription
        self.velocity_subscription = self.create_subscription(Float32, '/marv/nav/sbg_velocity_magnitude', self.marv_velocity_magnitude_callback, 10)
        self.velocity_subscription

        # Publishes

        # Allow time for subscriptions and publishes to be set up
        time.sleep(0.5)

        # Custom timers
        self.current_velocity_updated_timer.start()
        self.node_start_delay_timer.stop()

        # Timers
        # Set a timer for the main_loop_callback (where the scenario is running)
        self.main_loop_timer = self.create_timer(self.main_loop_update_period, self.main_loop_callback)
        self.get_new_params_timer = self.create_timer(self.get_new_params_period, self.update_params_callback)

    def __teardown(self):
        '''Teardown timers, publishers and subscriptions when the scenario is no longer running'''
        
        self.destroy_subscription(self.pose_subscription)
        self.destroy_subscription(self.velocity_subscription)
        #self.destroy_publisher(self.__node_state_publisher_)

        self.main_loop_timer.cancel()
        self.get_new_params_timer.cancel()

    ########################################################################################################

    ##################################### MAIN LOOP CALLBACK ###############################################

    # Main program loop
    def main_loop_callback(self):
        '''When the external mode is activated, get_scenario_running() will turn true. After that
            the steering commands needs to be updated with at least 10 Hz, using self.update_cmd_steering(), otherwise there will be
            a system error since the PDU will sense that the updates has stopped.
        '''

        if self.get_scenario_running():
            # Wait 2 seconds before starting
            if self.node_start_delay_timer.elapsed() == 0:
                self.node_start_delay_timer.start()
                self.finished_timer.reset()
                self.aps_signal = 0.0
                self.integral_term = 0.0

            # After 2 seconds start controller
            elif self.node_start_delay_timer.elapsed() > 2:
                # Calculate the error between ref and current vel
                self.error = (self.reference_velocity-self.current_velocity)
                # Update propitional term
                self.proportional_term = self.Kp*self.error
                # Update integral term
                self.integral_term += self.Ki*self.error
                # Update derivative term 
                self.derivative_term = self.Kd*(self.error-self.error_old)
                self.error_old = self.error
                # Update aps signal
                self.aps_signal = self.proportional_term + self.integral_term - self.derivative_term
                
                # Cap signal within reasonable limits
                if self.aps_signal > self.aps_signal_max:
                    self.aps_signal = self.aps_signal_max
                elif self.aps_signal < 0:
                    self.aps_signal = 0

                self.set_cmd_steering_aps(self.aps_signal)
            '''
            # Data too old, switch to fault state
            if self.current_velocity_updated_timer.elapsed() > self.current_velocity_update_timeout:
                self.set_state_fault()
                self.get_logger().info("FAULT: too old/no velocity input")'''
            
            # Check goal condition
            if self.current_velocity < self.reference_velocity + self.finished_velocity_intervl and self.current_velocity > self.reference_velocity - self.finished_velocity_intervl:
                if self.finished_timer.elapsed() == 0:
                    self.finished_timer.start()
                elif self.finished_timer.elapsed() > self.finished_waiting_time:
                    self.set_state_finished()
                    self.finished_timer.reset()
            else:
                self.finished_timer.reset()

            self.set_progress(str(round(self.finished_timer.elapsed(),2)) + "/" + str(self.finished_waiting_time)) # Show The time that we are within the goal velocity
            self.set_data_var_1(str(round(self.get_cmd_steering_aps(),2))) # Show APS
            self.set_data_var_2(str(round(self.current_velocity,2)) + "/" + str(self.reference_velocity)) # Show velocity

            self.update_cmd_steering() # Send the complete steering command to the system
            self.update_all_variables() # Send the updated variables to the system

            self.get_logger().info("APS: " + str(round(self.aps_signal,2)))

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    # Handle INS pose data
    def marv_pose_callback(self,ros_msg):
        print("handle INS pos data")

    # Get current velocity from INS
    def marv_velocity_magnitude_callback(self,ros_msg):
        self.current_velocity = ros_msg.data
        self.current_velocity_updated_timer.reset()

    def update_params_callback(self):
        self.Kp = self.get_parameter('throttle_aps.Kp').value
        self.Ki = self.get_parameter('throttle_aps.Ki').value 
        self.Kd = self.get_parameter('throttle_aps.Kd').value

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    test_constant_velocity = Test_Constant_Velocity()

    rclpy.spin(test_constant_velocity)

    test_constant_velocity.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
