#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV Example Scenario                 #
# By Viktor Lindstrom, Spring 2021      #
# Chalmers University of Technology     #
# ------------------------------------- #
# This is a simple example scenario to test if the the logic between all nodes functions as intended.
# It performs a simple task by starting up, then moving the handle to one side and then to the other.
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

# Messages
from geometry_msgs.msg import PoseWithCovariance

class Test_Throttle_Ramp(Scenario):

    ##################################### INIT #############################################################

    def __init__(self):
        '''The Init function is run on node start. Should not subscribe, publish or start any timers
            except for the "run_setup_check_timer". This is because we don't want anything running in the 
            background when this scenario is not running.
        '''
        # Scenario name parameter (this needs to be added/same as the name in /marv_driver/config/scenarios.yaml)
        __scenario_name = "TH_RAMP"
        # Scenario data variable headers, set to "" if not used, max 7 characters
        __data_var_1_header = "APS"
        __data_var_2_header = ""
        
        # Initialize Scenario Wrapper (handles the logic for starting and stopping scenarios)
        super().__init__('test_throttle_ramp',__scenario_name,__data_var_1_header,__data_var_2_header)

        self.get_logger().info("Initializing Scenario Node: " + __scenario_name)

        self.counter = 0
        self.aps_signal = 0

        # Check setup timer config
        self.__run_setup_check_timer_period = 0.1 # seconds between checking if the setup should be run (which should be done when the scenario is running)

        # Check setup timer
        self.__run_setup_check_timer = self.create_timer(self.__run_setup_check_timer_period,self.__run_setup_check_callback)

        self.get_logger().info("Started Scenario Node: " + __scenario_name)

        # Here it is possible to add anything else that needs to be initialized beforehand


    ########################################################################################################

    ##################################### SETUP AND CHECK ##################################################

    def __run_setup_check_callback(self):
        # If scenario is being initialized, run setup
        if self.get_scenario_state() == "INITIALIZING":
            self.__run_setup_check_timer.cancel() # Stop this timer, preventing the setup from being run several times
            self.__setup() # Call setup

    def __setup(self):
        '''Setup configuration parameters, global variables, subscriptions, publishes and timers.
            This function is run when the scenario is initializing, i.e. requested to start but not 
            yet runnung (we are still in manual mode which needs to be changed to external to start)
        '''
        # Configuration parameters 
        self.main_loop_update_period = 0.01 # seconds update time for the main loop, where the controller should be

        # Global variables
        self.node_test_timer = Timer()

        # Subscriptions (subscribe to for example sensor data)
        self.subscription = self.create_subscription(PoseWithCovariance, '/marv/nav/sbg_pose', self.marv_pose_callback, 10)
        self.subscription

        # Publishes

        self.counter = 0
        self.aps_signal = 0

        # Allow time for subscriptions and publishes to be set up
        time.sleep(0.5)

        # Timers
        # Set a timer for the main_loop_callback (where the scenario is running)
        self.main_loop_timer = self.create_timer(self.main_loop_update_period, self.main_loop_callback)

    ########################################################################################################

    ##################################### MAIN LOOP CALLBACK ###############################################

    # Main program loop
    def main_loop_callback(self):
        '''When the external mode is activated, get_scenario_running() will turn true. After that
            the steering commands needs to be updated with at least 10 Hz, using self.update_cmd_steering(), otherwise there will be
            a system error since the PDU will sense that the updates has stopped.
        '''
        
        if self.get_scenario_running():

            self.counter += 1
            if self.counter == 5:
                self.counter = 0
                self.aps_signal += 1
                if self.aps_signal > 100:
                    self.aps_signal = 0

            self.set_cmd_steering_aps(self.aps_signal)
            self.set_progress("1337")
            self.get_logger().info("cmd_throttle: " + str(self.get_cmd_steering_aps()))

            self.set_data_var_1(str(round(self.get_cmd_steering_aps(),2))) # Update data variable 1, shown on the OCU

            self.update_cmd_steering() # Send the complete steering command to the system
            self.update_all_variables() # Send the updated variables to the system

        else:
            self.node_test_timer.stop()

    # Handle INS pose data
    def marv_pose_callback(self,ros_msg):
        print("handle INS pos data")

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    test_throttle_ramp = Test_Throttle_Ramp()

    rclpy.spin(test_throttle_ramp)

    test_throttle_ramp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
