#!/usr/bin/env python3
''' 
# ------------------------------------------- #
# MARV Heading Controller Test Scenario       #
# By Viktor Lindstrom and                     #
# Noel Danielsson, Summer 2021                #
# Chalmers University of Technology           #
# ------------------------------------------- #
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
from heading_controller import HeadingController

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

# Messages
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool


class Heading_Controller_Test(Scenario):

    ##################################### INIT #############################################################

    def __init__(self):
        '''The Init function is run on node start. Should not subscribe, publish or start any timers
            except for the "run_setup_check_timer". This is because we don't want anything running in the 
            background when this scenario is not running.
        '''
        # Scenario name parameter (this needs to be added/same as the name in /marv_driver/config/scenarios.yaml)
        __scenario_name = "HED_CTL"
        # Scenario data variable headers, set to "" if not used, max 7 characters
        __data_var_1_header = "HEA_REF"
        __data_var_2_header = "HEA_CUR"

        # Initialize Scenario Wrapper (handles the logic for starting and stopping scenarios)
        super().__init__('heading_controller_test',__scenario_name,__data_var_1_header,__data_var_2_header)

        self.get_logger().info("Initializing Scenario Node: " + __scenario_name)

        # Declare rosparams
        self.declare_parameters(
            namespace='',
            parameters=[
                ('heading_controller_gain', None),
                ('heading_anti_windup_gain', None),
                ('steering_angle_limit', None),
                ('steering_angle_rate_limit', None),
                ('update_freq', None),
                ('wait_time', None),
                ('start_sound_time', None),
                ('heading_reference', None),
                ('ref_pos', None),
                ('aps_signal_ref', None),
            ])

        # ------- Global variables -------
        self.run_setup = True # Whenever to run the setup method when in INITIALIZING mode

        # Timers
        self.node_start_delay_timer = Timer()
        self.u_vel_update_timeout = 1 # second before counting the velocity input data as too outdated, signaling fault
        
        # State variables
        self.r_vel = 0.0 # Current angular velocity
        self.eta = np.array([0.0, 0.0, 0.0]) # Current position state: x,y,psi

        # Controller signal
        self.steering_angle = 0.0
        self.aps_signal = 0.0

        # Waypoint algorithm variables
        self.start_pos_set = False

        # --------------------------------

        # ------- Constants --------------
        # Heading and steering constants
        self.heading_controller_gain = self.get_parameter('heading_controller_gain').value
        self.heading_anti_windup_gain = self.get_parameter('heading_anti_windup_gain').value
        self.steering_angle_limit = self.get_parameter('steering_angle_limit').value
        self.steering_rate_limit = self.get_parameter('steering_angle_rate_limit').value

        # Other parameters
        self.update_freq = self.get_parameter('update_freq').value
        self.wait_time = self.get_parameter('wait_time').value
        self.start_sound_time = self.get_parameter('start_sound_time').value
        self.heading_reference = self.get_parameter('heading_reference').value
        self.aps_signal_ref = self.get_parameter('aps_signal_ref').value
        self.ref_pos = self.ref_pos = self.get_parameter('ref_pos').value
        # --------------------------------

        # Configuration parameters 
        self.main_loop_update_period = 1/self.update_freq # seconds update time for the main loop, where the controller should be
        self.get_new_params_period = 5.0 # seconds update time for updating with new rosparams

        # Load heading refshape fir coefficients
        hrf_file_path = os.getcwd() + '/colcon_ws/src/marv_scenarios/resource/heading_fir.csv'
        self.heading_ref_fir_coefficients = np.loadtxt(hrf_file_path, delimiter = ',')

        # Load heading yaw fir coefficients
        hyf_file_path = os.getcwd() + '/colcon_ws/src/marv_scenarios/resource/yaw_fir.csv'
        self.heading_yaw_fir_coefficients = np.loadtxt(hyf_file_path, delimiter = ',')

        # Other variables
        self.start_sound_sent = False
        self.start_sound_signal_id = 4
        self.controllers_created = False # Run controller setup once per run

        # Variables for finished condition (to determine if this scenario has succeeded
        self.finished_timer = Timer()
        self.finished_deg_intervl = 8.0 # +/- velocity where we count the scenario as finished after a certain time
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
        self.pose_subscription = self.create_subscription(PoseWithCovariance, '/marv/nav/sbg_pose', self.marv_pose_callback, 10)
        self.pose_subscription
        self.angular_velocity_subscription = self.create_subscription(Vector3, '/marv/nav/sbg_angular_velocity', self.marv_angular_velocity_callback, 10)
        self.angular_velocity_subscription
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
        
        self.destroy_subscription(self.pose_subscription)
        self.destroy_subscription(self.angular_velocity_subscription)
        self.destroy_subscription(self.ref_pos_state_subscription)
        self.destroy_publisher(self.ref_pos_publisher_)

        # Reset waypoint algorithm
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

            # Start
            elif self.node_start_delay_timer.elapsed() > self.wait_time:
                
                # Create controller objects
                if self.controllers_created == False:
                    self.controllers_created = True

                    self.heading_ctrl = HeadingController(  self.heading_ref_fir_coefficients,
                                                            self.heading_yaw_fir_coefficients,
                                                            self.steering_rate_limit,
                                                            self.update_freq,
                                                            self.heading_controller_gain,
                                                            np.deg2rad(self.steering_angle_limit),
                                                            self.heading_anti_windup_gain,
                                                            self.eta[2])
                    self.get_logger().info("Controller created, now starting")
                
                # Get new control signals
                self.aps_signal = self.aps_signal_ref

                if self.node_start_delay_timer.elapsed() > self.wait_time + 5:
                    self.steering_angle = self.heading_ctrl.run_controller(np.deg2rad(self.heading_reference), np.deg2rad(self.eta[2]), self.r_vel)
                else:
                    self.steering_angle = 0.0

                # Set steering commands
                self.set_cmd_steering_aps(self.aps_signal)
                self.set_cmd_steering_angle(self.steering_angle)

                self.set_progress(str(round(self.finished_timer.elapsed(),2)) + "/" + str(self.finished_waiting_time)) # Show The time that we are within the goal velocity
                self.set_data_var_1(str(round(self.heading_reference))) # Show heading reference
                self.set_data_var_2(str(round(self.eta[2]))) # Show current heading

            # Check goal condition
            if self.eta[2] < np.deg2rad(self.heading_reference) + np.deg2rad(self.finished_deg_intervl) and self.eta[2] > np.deg2rad(self.heading_reference) - np.deg2rad(self.finished_deg_intervl):
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

            self.get_logger().info("CUR_HEA: " + str(round(self.eta[2])) + ", REF_HEA: " + str(round(self.heading_reference)))

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    # Get current velocity from INS
    def marv_velocity_callback(self,ros_msg):
        self.u_vel = ros_msg.x # Surge

    def marv_angular_velocity_callback(self,ros_msg):
        self.r_vel = ros_msg.z # Yaw

    # Get current pose from INS
    def marv_pose_callback(self,ros_msg):
        self.eta[0] = ros_msg.pose.position.x
        self.eta[1] = ros_msg.pose.position.y
        eta_pose_quat = self.quaternion_to_euler_deg(ros_msg.pose.orientation.x, ros_msg.pose.orientation.y, ros_msg.pose.orientation.z, ros_msg.pose.orientation.w)
        self.eta[2] = eta_pose_quat[2]
        #self.get_logger().info("POS: " + str(self.eta))

    # Set reference position
    def marv_ref_pos_state_callback(self,ros_msg):
        if ros_msg.data == False:
            ref_pos_message = Vector3()
            ref_pos_message.x = float(self.ref_pos[0])
            ref_pos_message.y = float(self.ref_pos[1])
            ref_pos_message.z = float(self.ref_pos[2])
            self.ref_pos_publisher_.publish(ref_pos_message)

    def update_params_callback(self):
        # Heading and steering constants
        self.heading_controller_gain = self.get_parameter('heading_controller_gain').value
        self.heading_anti_windup_gain = self.get_parameter('heading_anti_windup_gain').value
        self.steering_angle_limit = self.get_parameter('steering_angle_limit').value
        self.steering_rate_limit = self.get_parameter('steering_angle_rate_limit').value

        # Other parameters
        self.update_freq = self.get_parameter('update_freq').value
        self.wait_time = self.get_parameter('wait_time').value
        self.start_sound_time = self.get_parameter('start_sound_time').value
        self.heading_reference = self.get_parameter('heading_reference').value
        self.aps_signal = self.get_parameter('aps_signal_ref').value
        self.ref_pos = self.ref_pos = self.get_parameter('ref_pos').value

    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################

    #Convert quaternions to roll pitch yaw angles in deg
    def quaternion_to_euler_deg(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.rad2deg(np.arctan2(sinr_cosp, cosr_cosp))

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >=1:
            pitch = np.rad2deg(np.sign(sinp) * np.pi/2)
        else:
            pitch = np.rad2deg(np.arcsin(sinp))

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.rad2deg(np.arctan2(siny_cosp, cosy_cosp))

        return roll, pitch, yaw

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    heading_controller_test = Heading_Controller_Test()

    rclpy.spin(heading_controller_test)

    heading_controller_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
