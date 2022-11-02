#!/usr/bin/env python3
''' 
# --------------------------------------- #
# MARV Remote Network Steering Scenario   #
# By Linus Johansson and                  #
# Joakim Osterman, Spring 2022            #
# Chalmers University of Technology       #
# --------------------------------------- #
'''

# System imports
import time
import numpy as np
import os

import json

# Custom libraries
from timer import Timer
from scenario import Scenario
from thrust_controller import ThrustController
from heading_controller import HeadingController

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

# Messages
from std_msgs.msg import Bool
from std_msgs.msg import String
from marv_msgs.msg import CmdSteering


class Remote_Network_Steering(Scenario):

    ##################################### INIT #############################################################

    def __init__(self):
        '''The Init function is run on node start. Should not subscribe, publish or start any timers
            except for the "run_setup_check_timer". This is because we don't want anything running in the 
            background when this scenario is not running.
        '''
        # Scenario name parameter (this needs to be added/same as the name in /marv_driver/config/scenarios.yaml)
        __scenario_name = "RNS"
        # Scenario data variable headers, set to "" if not used, max 7 characters
        __data_var_1_header = "SPEED"
        __data_var_2_header = "ANGLE"

        # Initialize Scenario Wrapper (handles the logic for starting and stopping scenarios)
        super().__init__('remote_network_steering',__scenario_name,__data_var_1_header,__data_var_2_header)

        self.get_logger().info("Initializing Scenario Node: " + __scenario_name)

        # Declare rosparams
        self.declare_parameters(
            namespace='',
            parameters=[
                ('thrust_controller_gain', [0.0, 0.0]),
                ('thrust_anti_windup_gain', 0.0),
                ('aps_signal_limit_min', 0.0),
                ('aps_signal_limit_max', 0.0),
                ('surge_rate_limit', 0.0),
                ('heading_controller_gain', [0.0, 0.0, 0.0]),
                ('heading_anti_windup_gain', 0.0),
                ('steering_angle_limit', 0),
                ('steering_angle_rate_limit', 0.0),
                ('u_min', 0.0),
                ('delta_t', 0.0),
                ('vel_diff_ang', 0.0),
                ('ref_pos', [0.0, 0.0, 0.0]),
                ('wp_file_name', "NOT_DEF"),
                ('update_freq', 0.0),
                ('wait_time', 0.0),
                ('start_sound_time', 0.0),
            ])

        # ------- Global variables -------
        self.run_setup = True # Whenever to run the setup method when in INITIALIZING mode

        # Timers
        self.node_start_delay_timer = Timer()
        self.u_vel_update_timeout = 1 # second before counting the velocity input data as too outdated, signaling fault
        
        # State variables
        #self.u_vel = 0.0 # Current velocity
        #self.r_vel = 0.0 # Current angular velocity
        #self.eta = np.array([0, 0, 0]) # Current position state: x,y,psi

        # Controller signal
        self.aps_signal = 0.0
        self.rps_signal = 0.0
        self.steering_angle = 0.0
        # --------------------------------

        # ------- Constants --------------
        # Thrust, APS and surge constants
        self.thrust_controller_gain = self.get_parameter('thrust_controller_gain').value
        self.thrust_anti_windup_gain = self.get_parameter('thrust_anti_windup_gain').value
        self.aps_signal_limit_min = self.get_parameter('aps_signal_limit_min').value
        self.aps_signal_limit_max = self.get_parameter('aps_signal_limit_max').value
        self.surge_rate_limit = self.get_parameter('surge_rate_limit').value

        # Heading and steering constants
        self.heading_controller_gain = self.get_parameter('heading_controller_gain').value
        self.heading_anti_windup_gain = self.get_parameter('heading_anti_windup_gain').value
        self.steering_angle_limit = self.get_parameter('steering_angle_limit').value
        self.steering_rate_limit = self.get_parameter('steering_angle_rate_limit').value

        # Other parameters
        self.u_min = self.get_parameter('u_min').value
        self.delta_t = self.get_parameter('delta_t').value
        self.vel_diff_ang = self.get_parameter('vel_diff_ang').value
        self.ref_pos = self.ref_pos = self.get_parameter('ref_pos').value
        self.wp_file_name = self.get_parameter('wp_file_name').value
        self.update_freq = self.get_parameter('update_freq').value
        self.wait_time = self.get_parameter('wait_time').value
        self.start_sound_time = self.get_parameter('start_sound_time').value
        # --------------------------------

        # Configuration parameters 
        self.main_loop_update_period = 1.0/self.update_freq # seconds update time for the main loop, where the controller should be
        self.get_new_params_period = 5.0 # seconds update time for updating with new rosparams
        
        # Load heading refshape fir coefficients
        hrf_file_path = os.getcwd() + '/colcon_ws/src/marv_scenarios/resource/heading_fir.csv'
        self.heading_ref_fir_coefficients = np.loadtxt(hrf_file_path, delimiter = ',')

        # Load heading yaw fir coefficients
        hyf_file_path = os.getcwd() + '/colcon_ws/src/marv_scenarios/resource/yaw_fir.csv'
        self.heading_yaw_fir_coefficients = np.loadtxt(hyf_file_path, delimiter = ',')

        # Load thrust fir coefficients
        tf_file_path = os.getcwd() + '/colcon_ws/src/marv_scenarios/resource/thrust_fir.csv'
        self.thrust_fir_coefficients = np.loadtxt(tf_file_path, delimiter = ',')

        # Other variables
        self.start_sound_sent = False
        self.start_sound_signal_id = 4
        self.controllers_created = False # Run controller setup once per run


        ### RNS Init stuff ###
        self.aps_max = 0.2
        self.rps_max = 0.1


        self.gamepad = {}

        self.killswitch = 0
        self.reset_killswitch = 0 
        
        #self.aps_signal = 0.0
        #self.steering_angle = 0.0

        self.dt = 0.03

        self.feedback = String()
        

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
        self.gamepad_input = self.create_subscription(String, 'web_ui/gamepad', self.receive_input, 10)

        # Publishes
        self.feedback_publisher = self.create_publisher(String, 'web_ui/feedback', 10)

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
        #self.get_new_params_timer = self.create_timer(self.get_new_params_period, self.update_params_callback)

    def __teardown(self):
        '''Teardown timers, publishers and subscriptions when the scenario is no longer running'''
        
        self.destroy_subscription(self.gamepad_input)
        self.destroy_publisher(self.feedback_publisher)

        self.set_cmd_steering_aps(0.0)
        self.set_cmd_steering_angle(0.0)

        self.main_loop_timer.cancel()
        #self.get_new_params_timer.cancel()

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
                self.rps_signal = 0.0
                self.steering_angle = 0.0
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

            # Start of main loop
            elif self.node_start_delay_timer.elapsed() > self.wait_time:
                
                ### Kill switch activation
                if self.gamepad.get("button0"):
                    self.killswitch = 1
                if self.gamepad.get("button3"):
                    self.reset_killswitch = 0
                if not self.gamepad.get("button0"):
                    self.killswitch = 0
                    self.reset_killswitch = 1

                ### Send steering commands if kill switch is held down
                if self.killswitch and not self.reset_killswitch:
                    self.steering_angle = -float(self.gamepad.get("lJoy")) * 22     # Jetski positive angle = left steer

                    lTrigVal = float(self.gamepad.get("lTrig")) * self.rps_max
                    rTrigVal = float(self.gamepad.get("rTrig")) * self.aps_max

                    ### Slowly ramp up aps/rps instead of sending raw input
                    if rTrigVal > self.aps_signal and self.aps_signal < self.aps_max and self.rps_signal == 0.0:
                        self.aps_signal += 0.01 
                    if rTrigVal < self.aps_signal and self.aps_signal > 0.00:
                        self.aps_signal -= 0.01
                    

                    if lTrigVal > self.rps_signal and self.rps_signal < self.rps_max and self.aps_signal == 0.0:
                        self.rps_signal += 0.01
                    if lTrigVal < self.rps_signal and self.rps_signal > 0.00:
                        self.rps_signal -= 0.01
       
                    
                ### Ramp down when kill switch released, make non-blocking
                if not self.killswitch:
                    if (self.aps_signal != 0.00 or self.rps_signal != 0.00):
                        if self.aps_signal > 0.00:
                            self.aps_signal -= 0.01
                        elif self.rps_signal > 0.00:
                            self.rps_signal -= 0.01

                ### Print command values, for debugging
                #print(f"APS: {self.aps_signal}, RPS: {self.rps_signal}, Angle: {self.steering_angle}")


                self.aps_signal = round(self.aps_signal, 2)
                self.rps_signal = round(self.rps_signal, 2)

                
                ### Send feedback to the web operator
                self.feedback.data = f"APS: {self.aps_signal}, RPS: {self.rps_signal}, Angle: {self.steering_angle}"
                self.feedback_publisher.publish(self.feedback)  


                # Set steering commands, aps and rps cannot be set simultaneously
                if self.aps_signal > 0.00:
                    self.set_cmd_steering_aps(self.aps_signal * 100)
                else:
                    self.set_cmd_steering_rps(self.rps_signal * 100)

                self.set_cmd_steering_angle(self.steering_angle)

            self.update_cmd_steering() # Send the complete steering command to the system
            self.update_all_variables() # Send the updated variables to the system

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    def receive_input(self, msg):
        try:
            self.gamepad = json.loads(msg.data)
        except:
            None

    def update_params_callback(self):
        # Thrust, APS and surge constants
        self.thrust_controller_gain = self.get_parameter('thrust_controller_gain').value
        self.thrust_anti_windup_gain = self.get_parameter('thrust_anti_windup_gain').value
        self.aps_signal_limit_min = self.get_parameter('aps_signal_limit_min').value
        self.aps_signal_limit_max = self.get_parameter('aps_signal_limit_max').value
        self.surge_rate_limit = self.get_parameter('surge_rate_limit').value

        # Heading and steering constants
        self.heading_controller_gain = self.get_parameter('heading_controller_gain').value
        self.heading_anti_windup_gain = self.get_parameter('heading_anti_windup_gain').value
        self.steering_angle_limit = self.get_parameter('steering_angle_limit').value
        self.steering_rate_limit = self.get_parameter('steering_angle_rate_limit').value

        # Other parameters
        self.u_min = self.get_parameter('u_min').value
        self.delta_t = self.get_parameter('delta_t').value
        self.vel_diff_ang = self.get_parameter('vel_diff_ang').value
        self.ref_pos = self.get_parameter('ref_pos').value
        self.wp_file_name = self.get_parameter('wp_file_name').value
        self.update_freq = self.get_parameter('update_freq').value
        self.wait_time = self.get_parameter('wait_time').value
        self.start_sound_time = self.get_parameter('start_sound_time').value


    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    remote_network_steering = Remote_Network_Steering()

    rclpy.spin(remote_network_steering)

    remote_network_steering.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()