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
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class Set_Reference_Position(Scenario):

    ##################################### INIT #############################################################

    def __init__(self):
        '''The Init function is run on node start. Should not subscribe, publish or start any timers
            except for the "run_setup_check_timer". This is because we don't want anything running in the 
            background when this scenario is not running.
        '''
        # Scenario name parameter (this needs to be added/same as the name in /marv_driver/config/scenarios.yaml)
        __scenario_name = "REF_POS"
        # Scenario data variable headers, set to "" if not used, max 7 characters
        __data_var_1_header = "STATE"
        __data_var_2_header = ""

        # Initialize Scenario Wrapper (handles the logic for starting and stopping scenarios)
        super().__init__('set_reference_position',__scenario_name,__data_var_1_header,__data_var_2_header)
    
        self.get_logger().info("Initializing Scenario Node: " + __scenario_name)

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
        self.ref_pos_lat = 57.6667 # Latitude
        self.ref_pos_lon = 11.8333 # Longitude

        # Global variables
        self.node_test_timer = Timer()
        self.ref_pos_set = False

        # Subscriptions (subscribe to for example sensor data)
        self.subscription = self.create_subscription(Bool, '/marv/nav/sbg_ref_pos_state', self.marv_ref_pos_state_callback, 10)
        self.subscription

        # Publishes
        self.marv_nav_sbg_ref_pos_publisher_ = self.create_publisher(Vector3, '/marv/nav/sbg_ref_pos', 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

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
            if not self.ref_pos_set:
                self.ref_pos_set = True

                # Publish reference position
                ref_pos = Vector3()
                ref_pos.x = self.ref_pos_lat
                ref_pos.y = self.ref_pos_lon 
                self.marv_nav_sbg_ref_pos_publisher_.publish(ref_pos)

                self.get_logger().info("Reference pos set")

            self.update_cmd_steering() # Send the complete steering command to the system
            self.update_all_variables() # Send the updated variables to the system

        else:
            self.node_test_timer.stop()

    def marv_ref_pos_state_callback(self,ros_msg):
        if ros_msg.data:
            self.set_data_var_1("SET")
        else:
            self.set_data_var_1("NOT_SET")

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    set_reference_position = Set_Reference_Position()

    rclpy.spin(set_reference_position)

    set_reference_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
