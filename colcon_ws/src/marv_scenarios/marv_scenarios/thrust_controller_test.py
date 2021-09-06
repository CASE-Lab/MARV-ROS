#!/usr/bin/env python3
''' 
# ------------------------------------------ #
# MARV Thrust Controller Test Scenario       #
# By Viktor Lindstrom and                    #
# Noel Danielsson, Summer 2021               #
# Chalmers University of Technology          #
# ------------------------------------------ #
'''

# System imports
import time
import numpy as np
import os

# Custom libraries
from timer import Timer
from scenario import Scenario
from waypoint_algorithm import Waypoint_Algorithm as wpa
from thrust_controller import ThrustController

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

# Messages
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool


class Thrust_Controller_Test(Scenario):

    ##################################### INIT #############################################################

    def __init__(self):
        '''The Init function is run on node start. Should not subscribe, publish or start any timers
            except for the "run_setup_check_timer". This is because we don't want anything running in the 
            background when this scenario is not running.
        '''
        # Scenario name parameter (this needs to be added/same as the name in /marv_driver/config/scenarios.yaml)
        __scenario_name = "THR_CTL"
        # Scenario data variable headers, set to "" if not used, max 7 characters
        __data_var_1_header = "APS"
        __data_var_2_header = "VEL"

        # Initialize Scenario Wrapper (handles the logic for starting and stopping scenarios)
        super().__init__('thrust_controller_test',__scenario_name,__data_var_1_header,__data_var_2_header)

        self.get_logger().info("Initializing Scenario Node: " + __scenario_name)

        # Declare rosparams
        self.declare_parameters(
            namespace='',
            parameters=[
                ('thrust_controller_gain', None),
                ('thrust_anti_windup_gain', None),
                ('aps_signal_limit_min', None),
                ('aps_signal_limit_max', None),
                ('surge_rate_limit', None),
                ('update_freq', None),
                ('wait_time', None),
                ('start_sound_time', None),
                ('reference_velocity', None),
                ('ref_pos', None),
            ])

        # ------- Global variables -------
        self.run_setup = True # Whenever to run the setup method when in INITIALIZING mode

        # Timers
        self.node_start_delay_timer = Timer()
        self.u_vel_update_timeout = 1 # second before counting the velocity input data as too outdated, signaling fault
        
        # State variables
        self.u_vel = 0.0 # Current velocity

        # Waypoint algorithm variables
        self.start_pos_set = False

        # Controller signal
        self.aps_signal = 0.0
        self.steering_angle = 0.0

        # Waypoint algorithm variables
        self.start_pos_set = False

        # --------------------------------

        # ------- Constants --------------
        # Thrust, APS and surge constants
        self.thrust_controller_gain = self.get_parameter('thrust_controller_gain').value
        self.thrust_anti_windup_gain = self.get_parameter('thrust_anti_windup_gain').value
        self.aps_signal_limit_min = self.get_parameter('aps_signal_limit_min').value
        self.aps_signal_limit_max = self.get_parameter('aps_signal_limit_max').value
        self.surge_rate_limit = self.get_parameter('surge_rate_limit').value

        # Other parameters
        self.update_freq = self.get_parameter('update_freq').value
        self.wait_time = self.get_parameter('wait_time').value
        self.start_sound_time = self.get_parameter('start_sound_time').value
        self.reference_velocity = self.get_parameter('reference_velocity').value
        self.ref_pos = self.ref_pos = self.get_parameter('ref_pos').value
        # --------------------------------

        # Configuration parameters 
        self.main_loop_update_period = 1/self.update_freq # seconds update time for the main loop, where the controller should be
        self.get_new_params_period = 5.0 # seconds update time for updating with new rosparams

        # Load thrust fir coefficients
        tf_file_path = os.getcwd() + '/colcon_ws/src/marv_scenarios/resource/thrust_fir.csv'
        self.thrust_fir_coefficients = np.loadtxt(tf_file_path, delimiter = ',')

        # Other variables
        self.start_sound_sent = False
        self.start_sound_signal_id = 4
        self.controllers_created = False # Run controller setup once per run

        # Variables for finished condition (to determine if this scenario has succeeded
        self.finished_timer = Timer()
        self.finished_velocity_intervl = 0.3 # +/- velocity where we count the scenario as finished after a certain time
        self.finished_waiting_time = 5 # seconds to wait before counting the scenario as finished if we are within the velocity interval

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
        self.velocity_subscription = self.create_subscription(Vector3, '/marv/nav/sbg_velocity', self.marv_velocity_callback, 10)
        self.velocity_subscription
        self.ref_pos_state_subscription = self.create_subscription(Bool, '/marv/nav/sbg_ref_pos_state', self.marv_ref_pos_state_callback, 10)
        self.ref_pos_state_subscription

        # Publishes
        self.ref_pos_publisher_ = self.create_publisher(Vector3, '/marv/nav/sbg_ref_pos', 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

        # Custom timers
        self.node_start_delay_timer.stop()

        # Other variables
        self.start_sound_sent = False
        self.controllers_created = False

        # Timers
        # Set a timer for the main_loop_callback (where the scenario is running)
        self.main_loop_timer = self.create_timer(self.main_loop_update_period, self.main_loop_callback)
        self.get_new_params_timer = self.create_timer(self.get_new_params_period, self.update_params_callback)

    def __teardown(self):
        '''Teardown timers, publishers and subscriptions when the scenario is no longer running'''
        
        self.destroy_subscription(self.velocity_subscription)
        self.destroy_subscription(self.ref_pos_state_subscription)
        self.destroy_publisher(self.ref_pos_publisher_)

        self.controllers_created = False
        self.start_pos_set = False

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
            # Start timer to wait before starting
            if self.node_start_delay_timer.elapsed() == 0:
                self.node_start_delay_timer.start()
                self.aps_signal = 0.0
                self.finished_timer.reset()
                self.get_logger().info("Timer started")

            # Wait
            elif self.node_start_delay_timer.elapsed() > 0 and self.node_start_delay_timer.elapsed() <= self.wait_time:
                if self.wait_time - self.node_start_delay_timer.elapsed() > 0:
                    self.set_progress(str(round(self.wait_time - self.node_start_delay_timer.elapsed(),2))) # Show start countdown
                else:
                    self.set_progress(str(0.00))

                self.set_data_var_1(str("--"))
                self.set_data_var_2(str("--"))

                # Send the buzzer start signal
                if self.start_sound_sent == False and (self.wait_time - self.node_start_delay_timer.elapsed() < self.start_sound_time) and self.start_sound_time != 0:
                    self.start_sound_sent = True
                    self.set_buzzer_signal(self.start_sound_signal_id)
                    self.update_buzzer_signal()
                    self.get_logger().info("Buzzer signal sent")

            # Start
            elif self.node_start_delay_timer.elapsed() > self.wait_time:
                
                # Create controller objects
                if self.controllers_created == False:
                    self.controllers_created = True
                    self.thrust_ctrl = ThrustController(    self.thrust_fir_coefficients,
                                                            self.surge_rate_limit,
                                                            self.update_freq,
                                                            self.thrust_controller_gain,
                                                            [self.aps_signal_limit_min, self.aps_signal_limit_max],
                                                            self.thrust_anti_windup_gain)

                    self.get_logger().info("Controller created, now starting")
                
                
                # Get new control signals
                self.aps_signal = self.thrust_ctrl.run_controller(self.reference_velocity, self.u_vel)

                # Set steering commands
                self.set_cmd_steering_aps(self.aps_signal)

                self.set_progress(str(round(self.finished_timer.elapsed(),2)) + "/" + str(self.finished_waiting_time)) # Show The time that we are within the goal velocity
                self.set_data_var_1(str(round(self.get_cmd_steering_aps(),2))) # Show APS
                self.set_data_var_2(str(round(self.u_vel,2)) + "/" + str(self.reference_velocity)) # Show velocity

            # Check goal condition
            if self.u_vel < self.reference_velocity + self.finished_velocity_intervl and self.u_vel > self.reference_velocity - self.finished_velocity_intervl:
                if self.finished_timer.elapsed() == 0:
                    self.finished_timer.start()
                elif self.finished_timer.elapsed() > self.finished_waiting_time:
                    self.set_state_finished()
                    self.finished_timer.reset()
                    self.get_logger().info("Scenario finished")
            else:
                self.finished_timer.reset()

            self.update_cmd_steering() # Send the complete steering command to the system
            self.update_all_variables() # Send the updated variables to the system

            self.get_logger().info("APS: " + str(round(self.aps_signal,2)))
            self.get_logger().info("VEL: " + str(round(self.u_vel,2)))

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    # Get current velocity from INS
    def marv_velocity_callback(self,ros_msg):
        self.u_vel = ros_msg.x # Surge

    # Set reference position
    def marv_ref_pos_state_callback(self,ros_msg):
        if ros_msg.data == False:
            ref_pos_message = Vector3()
            ref_pos_message.x = float(self.ref_pos[0])
            ref_pos_message.y = float(self.ref_pos[1])
            ref_pos_message.z = float(self.ref_pos[2])
            self.ref_pos_publisher_.publish(ref_pos_message)

    def update_params_callback(self):
        # Thrust, APS and surge constants
        self.thrust_controller_gain = self.get_parameter('thrust_controller_gain').value
        self.thrust_anti_windup_gain = self.get_parameter('thrust_anti_windup_gain').value
        self.aps_signal_limit_min = self.get_parameter('aps_signal_limit_min').value
        self.aps_signal_limit_max = self.get_parameter('aps_signal_limit_max').value
        self.surge_rate_limit = self.get_parameter('surge_rate_limit').value

        # Other parameters
        self.update_freq = self.get_parameter('update_freq').value
        self.wait_time = self.get_parameter('wait_time').value
        self.start_sound_time = self.get_parameter('start_sound_time').value
        self.reference_velocity = self.get_parameter('reference_velocity').value
        self.ref_pos = self.ref_pos = self.get_parameter('ref_pos').value

    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    thrust_controller_test = Thrust_Controller_Test()

    rclpy.spin(thrust_controller_test)

    thrust_controller_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
