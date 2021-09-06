#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV Scenario Handler node      #
# By Viktor Lindstrom and               #
# Noel Danielsson, Spring 2021          #
# Chalmers University of Technology     #
# ------------------------------------- #
# Waverunner Scenario Handler takes care of everything regarding configuration and executing scenarios.
# It sends scenario configuration to the OCU. Takes care of requests to start and stop scenarios. 
'''

# System imports
import numpy as np
import time
import datetime

# Custom libraries
from timer import Timer

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

# Messages
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt64
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import String
from marv_msgs.msg import CmdSteering
from marv_msgs.msg import ScenarioConfig
from marv_msgs.msg import Notification
from marv_msgs.msg import Status
from marv_msgs.msg import DataVarHeader

class MARV_Scenario_Handler(Node):

    ##################################### INIT #############################################################
    def __init__(self):
        # Setup parameters, variables, publishers, subscribers and timers
        super().__init__('marv_scenario_handler')

        self.get_logger().info("Initializing...")

        #print("Initializing...")

        # Load scenario parameters
        self.nbr_of_scenarios = 8
        self.scenario_header = [""] * self.nbr_of_scenarios
        self.scenario_data_header_1 = [""] * self.nbr_of_scenarios
        self.scenario_data_header_2 = [""] * self.nbr_of_scenarios

        for index in range(0,self.nbr_of_scenarios):
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('scenario_' + str(index) + '.name', None),
                ])
            self.scenario_header[index] = self.get_parameter('scenario_' + str(index) + '.name').value

        # Configuration parameters 
        self.scenario_config_message_period = 5.0 # seconds update time for publishing scenario configuration
        self.scenario_system_time_message_period = 0.2 # seconds update time for publishing the scenario system time
        self.scenario_state_update_message_period = 0.1 # seconds update time for state transitions and messages
        self.notification_message_length = 70 # max length of notification message in characters, needs to be in fractions of 7 (also needs to be set in OCU code)
        self.node_state_timer_Timeout = 0.1 # seconds before counting a node as not connected, if not state updates are recived 
        self.scenario_node_state_timeout_update_period = 0.1 # seconds update time to check is scenario node has timed out it's state update
        self.scenario_start_request_handle_waiting_period = 3.0 # seconds non blocking period, allows the scenario node to start up and respond
        self.scenario_handler_fault_finished_state_timeout_period = 0.02 # seconds between cmdSteering commands when in fault or finished state
        self.scenario_start_timeout_period = 30.0 # seconds timeout before transitioning back to stopped mode from waiting mode, prevents deadlocks
        self.heartbeat_timeout_time = 3.0 # seconds timeout time for counting a unit as timed out
        self.heartbeat_timeout_update_period = 0.2 # seconds update time for checking the heartbeat timeout
        self.current_scenario_publisher_update_period = 0.5 # seconds update time for publishing the current scenario

        # Rate limit variables, prevent the CAN bus from overflooding 
        self.rateLimit_cmdSteering = 0.02 # seconds. How often a command can be sent, any message recived withn the specidifed time will be ignored (i.e. 0.01s => at most 100 Hz update frequency)
        self.rateLimit_progress = 0.2
        self.rateLimit_data_1 = 0.5
        self.rateLimit_data_2 = 0.5
        self.rateLimit_send_notification = 1.0
        self.rateLimit_send_buzzer_signal = 1.0
        self.rateLimit_cmdSteering_timer = Timer()
        self.rateLimit_progress_timer = Timer()
        self.rateLimit_data_1_timer = Timer()
        self.rateLimit_data_2_timer = Timer()
        self.rateLimit_send_notification_timer = Timer()
        self.rateLimit_send_buzzer_signal_timer = Timer()

        # Global variables
        # Timers
        self.node_state_timer = Timer()

        # Scources
        self.source = {
            "NONE":         0,
            "RCU":          1,
            "ACU":          2,
            "TESTING":      3
        }

        # Scenario states
        self.handler_state = {
            "WAITING":      0,
            "EXECUTING":    1,
            "FAULT":        2,
            "STOPPED":      3,
            "FINISHED":     4,
            "REQ_SENT":     5
        }

        # Scenario node states
        self.node_state = {
            "NOT_CONNECTED": -1,
            "INITIALIZING": 0,
            "EXECUTING":    1,
            "FAULT":        2,
            "STOPPED":      3,
            "FINISHED":     4
        }

        # Control mode states, External or Manual mode
        self.mode_state = {
            "MAN":  0,
            "EXT":  1
        }

        self.heartbeat_state = {
            "ER":   0,
            "OK":   1,
            "NC":   2
        }

        self.scenario_handler_state = self.handler_state["STOPPED"]
        self.scenario_node_state = self.node_state["NOT_CONNECTED"]
        self.mode = self.mode_state["MAN"]
        self.scenario_running = False
        self.handling_scenario_start_request = False
        self.steering_cmd_aps = 0.0
        self.steering_cmd_rps = 0.0
        self.steering_cmd_angle = 0.0
        self.active_scenario = -1
        self.node_subscriptions_destroyed = True

        # TODO
        # Heartbeat variables
        self.pdu_state = self.heartbeat_state["NC"]
        self.tcu_state = self.heartbeat_state["NC"] 
        self.ncu_state = self.heartbeat_state["NC"]
        self.heartbeatTimer_pdu = Timer()
        self.heartbeatTimer_tcu = Timer()
        self.heartbeatTimer_ncu = Timer()

        # Subscribe to various topics
        self.subscription = self.create_subscription(Bool, '/marv/sys/status/state_12V_auto', self.mode_callback, 10)
        self.subscription = self.create_subscription(UInt8, '/marv/sys/status/start_scenario', self.scenario_start_request_callback, 10)
        self.subscription = self.create_subscription(DataVarHeader, '/marv/node/data_var_header', self.data_var_header_callback, 10)
        self.subscription = self.create_subscription(String, '/marv/sys/ctrl/notification_message', self.send_notification_callback, 10)
        self.subscription = self.create_subscription(Bool, '/marv/sys/ctrl/reset_scenario_handler', self.reset_scenario_handler_callback, 10)
        self.subscription = self.create_subscription(Status, '/marv/sys/status/heartbeat_pdu', self.heartbeat_pdu_callback, 10)
        self.subscription = self.create_subscription(Status, '/marv/sys/status/heartbeat_tcu', self.heartbeat_tcu_callback, 10)
        self.subscription = self.create_subscription(Status, '/marv/sys/status/heartbeat_ncu', self.heartbeat_ncu_callback, 10)

        # Publishes scenario configuration
        self.marv_sys_ctrl_scenario_header_publisher_ = self.create_publisher(ScenarioConfig, '/marv/sys/ctrl/scenario_header', 10)
        self.marv_sys_ctrl_scenario_data_header_publisher_ = self.create_publisher(ScenarioConfig, '/marv/sys/ctrl/scenario_data_header', 10)
        self.marv_sys_ctrl_scenario_data_publisher_ = self.create_publisher(ScenarioConfig, '/marv/sys/ctrl/scenario_data', 10)
        self.marv_sys_ctrl_scenario_config_publisher_ = self.create_publisher(ScenarioConfig, '/marv/sys/ctrl/scenario_config', 10)
        self.marv_sys_ctrl_scenario_sys_time_publisher_ = self.create_publisher(UInt64, '/marv/sys/ctrl/scenario_sys_time', 10)
        self.marv_sys_ctrl_scenario_state_publisher_ = self.create_publisher(Int8, '/marv/sys/ctrl/scenario_state', 10)
        self.marv_sys_ctrl_notification_message_publisher_ = self.create_publisher(Notification, '/marv/sys/ctrl/formatted_notification_message', 10)
        self.marv_sys_ctrl_cmdSteering_publisher_ = self.create_publisher(CmdSteering, '/marv/sys/ctrl/cmd_steering', 10)
        self.marv_sys_ctrl_log_data_publisher_ = self.create_publisher(Bool, '/marv/sys/ctrl/log_data', 10)
        self.marv_sys_ctrl_buzzer_signal_publisher_ = self.create_publisher(Int8, '/marv/sys/ctrl/sound_buzzer', 10)
        self.marv_sys_status_current_scenario_publisher_ = self.create_publisher(String, '/marv/sys/status/current_scenario', 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

        # Setup timers for periodic callbacks
        self.scenario_config_timer = self.create_timer(self.scenario_config_message_period, self.scenario_configuration_callback)
        #self.scenario_progress = self.create_timer(5.0,self.scenario_progress_test)
        self.scenario_system_time_timer = self.create_timer(self.scenario_system_time_message_period, self.scenario_sys_time_callback)
        self.scenario_state_update_timer = self.create_timer(self.scenario_state_update_message_period,self.scenario_state_update_callback)
        self.scenario_node_state_timeout_timer = self.create_timer(self.scenario_node_state_timeout_update_period,self.scenario_node_state_timeout_callback)
        #self.scenario_start_request_handle_timer = self.create_timer(self.scenario_start_request_handle_update_period,self.check_start_request_callback)
        self.scenario_start_request_handle_timer = self.create_timer(self.scenario_start_request_handle_waiting_period,self.scenario_start_request_handle_callback)
        self.scenario_start_request_handle_timer.cancel()
        self.scenario_handler_fault_finished_state_timer = self.create_timer(self.scenario_handler_fault_finished_state_timeout_period,self.scenario_handler_fault_finished_state_callback)
        self.scenario_handler_fault_finished_state_timer.cancel()
        self.scenario_start_timeout_timer = self.create_timer(self.scenario_start_timeout_period,self.scenario_start_timeout_callback)
        self.scenario_start_timeout_timer.cancel() 
        self.heartbeat_timeout_timer = self.create_timer(self.heartbeat_timeout_update_period,self.heartbeat_timeout_callback)
        self.current_scenario_publisher_timer = self.create_timer(self.current_scenario_publisher_update_period,self.current_scenario_publisher_callback)

        #print("Running...")

        self.get_logger().info("Running...")

    ########################################################################################################

    ##################################### CALLBACKS ########################################################
    
    # Scenario state transitions and update messages to OCU
    def scenario_state_update_callback(self):

        # State transitions
        if self.mode == self.mode_state["MAN"] and not self.scenario_handler_state == self.handler_state["WAITING"]:
            self.scenario_handler_state = self.handler_state["STOPPED"]
            
            # Stop scenario, reset scenario handler
            if not self.handling_scenario_start_request and not self.node_subscriptions_destroyed:
                self.get_logger().info("State: STOPPED")
                self.reset_scenario_handler()
                
        elif self.mode == self.mode_state["EXT"] and self.scenario_handler_state == self.handler_state["WAITING"]:
            self.scenario_handler_state = self.handler_state["EXECUTING"]
            self.get_logger().info("State: EXECUTING")
            self.scenario_start_timeout_timer.cancel()

        # Publish scenario state
        scenario_state_message = Int8()
        scenario_state_message.data = self.scenario_handler_state
        self.marv_sys_ctrl_scenario_state_publisher_.publish(scenario_state_message)

    # Scenario start request handler
    def scenario_start_request_callback(self,ros_msg):
        # TODO
        '''if self.pdu_state != self.heartbeat_state["OK"] or self.tcu_state != self.heartbeat_state["OK"] or self.ncu_state != self.heartbeat_state["OK"]:
            reverse_heartbeat_state = dict(map(reversed,self.heartbeat_state.items()))
            message_string = "COULD NOT#START# #NODE#STATUS:# #"
            message_string += "PDU: " + str(reverse_heartbeat_state[self.pdu_state]) + "#"
            message_string += "TCU: " + str(reverse_heartbeat_state[self.tcu_state]) + "#"
            message_string += "NCU: " + str(reverse_heartbeat_state[self.ncu_state]) + "#"
            
            self.send_notification(message_string)
            self.get_logger().info("Not in OK state...")'''

        # Remove this block and use the one above when the pdu CAN has been fixed
        if self.tcu_state != self.heartbeat_state["OK"] or self.ncu_state != self.heartbeat_state["OK"]:
            reverse_heartbeat_state = dict(map(reversed,self.heartbeat_state.items()))
            message_string = "COULD NOT#START# #NODE#STATUS:# #"
            message_string += "TCU: " + str(reverse_heartbeat_state[self.tcu_state]) + "#"
            message_string += "NCU: " + str(reverse_heartbeat_state[self.ncu_state]) + "#"
            
            self.send_notification(message_string)
            self.get_logger().info("Not in OK state...")

        elif not self.scenario_running and not self.handling_scenario_start_request:
            self.active_scenario = ros_msg.data
            self.handling_scenario_start_request = True
            self.node_subscriptions_destroyed = False

            # Handle node call and startup
            
            # Create node start request publisher
            scenario_node_start_request_TOPIC = "/marv/node/" + self.scenario_header[self.active_scenario] + "/start_scenario"
            self.node_scenario_start_request_publisher_ = self.create_publisher(Bool, scenario_node_start_request_TOPIC, 10)
            
            # Create node state subscription
            scenario_node_state_TOPIC = "/marv/node/" + self.scenario_header[self.active_scenario] + "/state"
            self.node_state_subscription = self.create_subscription(Int8, scenario_node_state_TOPIC, self.scenario_node_state_callback, 10)
            self.node_state_subscription
            
            # Create node cmdSteering subscription
            scenario_node_cmdSteering_subscription_TOPIC = "/marv/node/" + self.scenario_header[self.active_scenario] + "/cmdSteering"
            self.node_cmdSteering_subscription = self.create_subscription(CmdSteering, scenario_node_cmdSteering_subscription_TOPIC, self.scenario_node_cmdSteering_callback, 10)
            self.node_cmdSteering_subscription
            
            # Create node progress update,data1 var,data2 var and send notification subscriptions
            scenario_node_progress_update_subscription_TOPIC = "/marv/node/" + self.scenario_header[self.active_scenario] + "/progress_update"
            scenario_node_data_1_update_subscription_TOPIC = "/marv/node/" + self.scenario_header[self.active_scenario] + "/data_var_1_update"
            scenario_node_data_2_update_subscription_TOPIC = "/marv/node/" + self.scenario_header[self.active_scenario] + "/data_var_2_update"
            scenario_node_send_notification_subscription_TOPIC = "/marv/node/" + self.scenario_header[self.active_scenario] + "/send_notification"
            scenario_node_send_buzzer_signal_subscription_TOPIC = "/marv/node/" + self.scenario_header[self.active_scenario] + "/send_buzzer_signal"
            self.node_progress_update_subscription = self.create_subscription(String, scenario_node_progress_update_subscription_TOPIC, self.scenario_progress_update_callback, 10)
            self.node_data_1_update_subscription = self.create_subscription(String, scenario_node_data_1_update_subscription_TOPIC, self.scenario_data_1_update_callback, 10)
            self.node_data_2_update_subscription = self.create_subscription(String, scenario_node_data_2_update_subscription_TOPIC, self.scenario_data_2_update_callback, 10)
            self.node_send_notification_subscription = self.create_subscription(String, scenario_node_send_notification_subscription_TOPIC, self.send_notification_callback, 10)
            self.node_send_buzzer_signal_subscription = self.create_subscription(Int8, scenario_node_send_buzzer_signal_subscription_TOPIC, self.send_buzzer_signal_callback, 10)
            self.node_progress_update_subscription
            self.node_data_1_update_subscription
            self.node_data_2_update_subscription
            self.node_send_notification_subscription
            self.node_send_buzzer_signal_subscription

            # Allow time for subscriptions and publishes to be set up
            time.sleep(0.5)

            # Send start request to node
            start_message = Bool()
            start_message.data = True
            self.node_scenario_start_request_publisher_.publish(start_message)

            # Start timer to let the node handle the start request (nonblocking delay before evaulating if the scenario node has been started)
            self.scenario_start_request_handle_timer.reset()
            
            self.get_logger().info("Start req sent to node...")

    # Evaluate if the scenario node has been started
    def scenario_start_request_handle_callback(self):
        # If node has respoinded with STOPPED state and cmdSteering 0 deg andgle, 0 throttle, then the scenario is allowed to start
        if self.scenario_node_state == self.node_state["INITIALIZING"] and self.steering_cmd_aps == 0.0 and self.steering_cmd_rps == 0.0 and self.steering_cmd_angle == 0.0:
            # On confirmation
            self.scenario_handler_state = self.handler_state["WAITING"]
            self.scenario_start_timeout_timer.reset()
            self.scenario_running = True

            # Start logging
            logging_message = Bool()
            logging_message.data = True
            self.marv_sys_ctrl_log_data_publisher_.publish(logging_message)

            #print("State: WAITING")
            self.get_logger().info("State: WAITING")

        elif self.scenario_node_state == self.node_state["NOT_CONNECTED"]:
            self.send_notification("COULD NOT#START# #NODE NOT#CONNECTED#")
        elif not self.scenario_node_state == self.node_state["INITIALIZING"]:
            self.send_notification("COULD NOT#START# #NODE#NOT IN#INIT#STATE#")
        elif not self.steering_cmd_aps == 0 or not self.steering_cmd_rps == 0 or not self.steering_cmd_angle == 0:
            self.send_notification("COULD NOT#START# #NODE#CMD_STE#NOT ZERO#")

        # Stop timer, we only want this to run once every start request
        self.scenario_start_request_handle_timer.cancel()
        
        # We have handled the start request
        self.handling_scenario_start_request = False

        #print("Done handling start request...")
        self.get_logger().info("Done handling start request...")

    # Handles automatic transitioning back from WAITING to STOPPED mode, prevents deadlocks that can arise
    def scenario_start_timeout_callback(self):
        self.scenario_start_timeout_timer.cancel()

        # Send stop request to node
        stop_message = Bool()
        stop_message.data = False
        self.node_scenario_start_request_publisher_.publish(stop_message)

        self.get_logger().info("WAITING state timeout...")
        self.send_notification("REQUEST#TIMED OUT# #START#SCENARIO#AGAIN#")
        self.reset_scenario_handler()

    # Scenario node state update
    def scenario_node_state_callback(self,ros_msg):
        self.scenario_node_state = ros_msg.data

        # Start timer
        if self.node_state_timer.elapsed == 0:
            self.node_state_timer.start()
        else:
            self.node_state_timer.reset()

        # Give some authority to the scenario controller node, to set FAULT and FINISHED state
        if self.scenario_running:
            if self.mode == self.mode_state["EXT"] and self.scenario_node_state == self.node_state["FINISHED"]:
                self.scenario_handler_state = self.handler_state["FINISHED"]
                self.scenario_handler_fault_finished_state_timer.reset()
                self.scenario_running = False
                
                # Stop logging
                logging_message = Bool()
                logging_message.data = False
                self.marv_sys_ctrl_log_data_publisher_.publish(logging_message)

                #print("State: FINISHED")
                self.get_logger().info("State: FINISHED")

            elif self.mode == self.mode_state["EXT"] and self.scenario_node_state == self.node_state["FAULT"]:
                self.scenario_handler_state = self.handler_state["FAULT"]
                self.scenario_handler_fault_finished_state_timer.reset()
                self.scenario_running = False

                # Stop logging
                logging_message = Bool()
                logging_message.data = False
                self.marv_sys_ctrl_log_data_publisher_.publish(logging_message)

                #print("State: FAULT")
                self.get_logger().info("State: FAULT")

    # Mode callback, listens to the system 12V auto, activated => External mode, deactivated => Manual mode
    def mode_callback(self,ros_msg):
        if ros_msg.data == False:
            self.mode = self.mode_state["MAN"]
        else:
            self.mode = self.mode_state["EXT"]

    # Send the scenario configuration to the OCU
    def scenario_configuration_callback(self):
        # Set scenario headers
        self.scenario_configuration_update(0,self.scenario_header)
        # Set data headers
        self.scenario_configuration_update(2,self.scenario_data_header_1)
        # Set data headers
        self.scenario_configuration_update(4,self.scenario_data_header_2)

    # Updates the system time for the OCU
    def scenario_sys_time_callback(self):
        systemTime = datetime.datetime.now()
        time = datetime.datetime(systemTime.year,systemTime.month,systemTime.day)
        timeToday = systemTime.timestamp() - time.timestamp()

        timeToday_message = UInt64()
        timeToday_message.data = int(timeToday)
        self.marv_sys_ctrl_scenario_sys_time_publisher_.publish(timeToday_message)

    # Updates the progress string for the OCU
    def scenario_progress_update_callback(self,ros_msg):
        if self.rateLimit_progress_timer.elapsed() > self.rateLimit_progress:
            self.rateLimit_progress_timer.reset()

            if self.active_scenario != -1 and self.scenario_running:
                self.publish_scenario_configuration(self.active_scenario,1,ros_msg.data)

        elif self.rateLimit_progress_timer.elapsed() == 0:
            self.rateLimit_progress_timer.start()

            if self.active_scenario != -1 and self.scenario_running:
                self.publish_scenario_configuration(self.active_scenario,1,ros_msg.data)

    # Updates the data_1 variable for the OCU
    def scenario_data_1_update_callback(self,ros_msg):
        if self.rateLimit_data_1_timer.elapsed() > self.rateLimit_data_1:
            self.rateLimit_data_1_timer.reset()

            if self.active_scenario != -1 and self.scenario_running:
                self.publish_scenario_configuration(self.active_scenario,3,ros_msg.data)

        elif self.rateLimit_data_1_timer.elapsed() == 0:
            self.rateLimit_data_1_timer.start()

            if self.active_scenario != -1 and self.scenario_running:
                self.publish_scenario_configuration(self.active_scenario,3,ros_msg.data)

    # Updates the data_2 variable for the OCU
    def scenario_data_2_update_callback(self,ros_msg):
        if self.rateLimit_data_2_timer.elapsed() > self.rateLimit_data_2:
            self.rateLimit_data_2_timer.reset()

            if self.active_scenario != -1 and self.scenario_running:
                self.publish_scenario_configuration(self.active_scenario,5,ros_msg.data)

        elif self.rateLimit_data_2_timer.elapsed() == 0:
            self.rateLimit_data_2_timer.start()

            if self.active_scenario != -1 and self.scenario_running:
                self.publish_scenario_configuration(self.active_scenario,5,ros_msg.data)

    # Sends a notification message to the OCU, displayed on screen for 4 seconds, including sound from buzzer
    def send_notification_callback(self,ros_msg):
        if self.rateLimit_send_notification_timer.elapsed() > self.rateLimit_send_notification:
            self.rateLimit_send_notification_timer.reset()

            self.send_notification(ros_msg.data)

        elif self.rateLimit_send_notification_timer.elapsed() == 0:
            self.rateLimit_send_notification_timer.start()

            self.send_notification(ros_msg.data)

    # Publishes a buzzer signal to the UCU
    def send_buzzer_signal_callback(self,ros_msg):
        if self.rateLimit_send_buzzer_signal_timer.elapsed() > self.rateLimit_send_buzzer_signal:
            self.rateLimit_send_buzzer_signal_timer.reset()

            buzzer_signal_message = Int8()
            buzzer_signal_message.data = int(ros_msg.data)
            self.marv_sys_ctrl_buzzer_signal_publisher_.publish(buzzer_signal_message)

        elif self.rateLimit_send_buzzer_signal_timer.elapsed() == 0:
            self.rateLimit_send_buzzer_signal_timer.start()

            buzzer_signal_message = Int8()
            buzzer_signal_message.data = int(ros_msg.data)
            self.marv_sys_ctrl_buzzer_signal_publisher_.publish(buzzer_signal_message)

    # Set the node state to not connected if the update has timed out
    def scenario_node_state_timeout_callback(self):
        #print("Node state: " + str(self.scenario_node_state))
        #print("Timer: " + str(self.node_state_timer.elapsed()))
        if self.node_state_timer.elapsed() > self.node_state_timer_Timeout:
            self.scenario_node_state = self.node_state["NOT_CONNECTED"]
            self.node_state_timer.stop()
            
            self.get_logger().info("State: NOT_CONNECTED")
            #print("Stopped timer")
            self.get_logger().info("Stopped timer")
            
            # If the node stops sending state updates during execution, switch to FAULT state
            if self.scenario_running:
                self.scenario_handler_state = self.handler_state["FAULT"]
                self.scenario_handler_fault_finished_state_timer.reset()
                self.send_cmdSteering(0.0, 0.0, self.steering_cmd_angle)
                self.scenario_running = False

                self.get_logger().info("State: FAULT")

            # Stop logging
            logging_message = Bool()
            logging_message.data = False
            self.marv_sys_ctrl_log_data_publisher_.publish(logging_message)

    # cmdSteering callback from scenario node, if scenario active then it is relayed to the can bridge
    def scenario_node_cmdSteering_callback(self,ros_msg):
        if self.rateLimit_cmdSteering_timer.elapsed() > self.rateLimit_cmdSteering:
            self.rateLimit_cmdSteering_timer.reset()

            self.steering_cmd_aps = ros_msg.aps
            self.steering_cmd_rps = ros_msg.rps
            self.steering_cmd_angle = ros_msg.angle

            if self.scenario_running:
                self.send_cmdSteering(ros_msg.aps, ros_msg.rps, ros_msg.angle)

        elif self.rateLimit_cmdSteering_timer.elapsed() == 0:
            self.rateLimit_cmdSteering_timer.start()
        
    # Resets the scenario handler by a ros message
    def reset_scenario_handler_callback(self,ros_msg):
        self.get_logger().info("Resetted by ROS message")
        self.reset_scenario_handler()

    # Sends cmdSteering commands when scenario handler is in fault or finished state
    def scenario_handler_fault_finished_state_callback(self):
        if self.scenario_handler_state == self.handler_state["FAULT"] or self.scenario_handler_state == self.handler_state["FINISHED"]:
            self.send_cmdSteering(0.0, 0.0, self.steering_cmd_angle)
        else: # Does not run when in other states
            self.scenario_handler_fault_finished_state_timer.cancel()

    def heartbeat_pdu_callback(self,ros_msg):
        self.pdu_state = ros_msg.status
        self.heartbeatTimer_pdu.reset()

    def heartbeat_tcu_callback(self,ros_msg):
        self.tcu_state = ros_msg.status
        self.heartbeatTimer_tcu.reset()

    def heartbeat_ncu_callback(self,ros_msg):
        self.ncu_state = ros_msg.status
        self.heartbeatTimer_ncu.reset()

    def heartbeat_timeout_callback(self):
        if self.heartbeatTimer_pdu.elapsed() > self.heartbeat_timeout_time:
            self.pdu_state = self.heartbeat_state["NC"]
            self.heartbeatTimer_pdu.stop()
        if self.heartbeatTimer_tcu.elapsed() > self.heartbeat_timeout_time:
            self.tcu_state = self.heartbeat_state["NC"]
            self.heartbeatTimer_tcu.stop()
        if self.heartbeatTimer_ncu.elapsed() > self.heartbeat_timeout_time:
            self.ncu_state = self.heartbeat_state["NC"]
            self.heartbeatTimer_ncu.stop()

    # Update data var headers for a scenario
    def data_var_header_callback(self,ros_msg):
        for index in range(0,self.nbr_of_scenarios):
            if ros_msg.header == self.scenario_header[index]:
                self.scenario_data_header_1[index] = ros_msg.data_1_header
                self.scenario_data_header_2[index] = ros_msg.data_2_header

    # Publishes the current scenario, for logging
    def current_scenario_publisher_callback(self):
        current_scenario_message = String()
        if self.active_scenario != -1:
            current_scenario_message.data = self.scenario_header[self.active_scenario]
        else:
            current_scenario_message.data = "NO_SCENARIO"
        self.marv_sys_status_current_scenario_publisher_.publish(current_scenario_message)


    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################

    def scenario_configuration_update(self,location,data_list):
        for scenario_index in range(0,len(data_list)):
            if not data_list[scenario_index] == "":
                self.publish_scenario_configuration(scenario_index,location,data_list[scenario_index])
                time.sleep(0.001) # Prevent PDU CAN from crashing

    def publish_scenario_configuration(self,scenario_id,location,data):
        if len(list(data)) < 8:
            config_header_message = ScenarioConfig()
            config_header_message.scenario_id = scenario_id
            config_header_message.location = location # 0 for header, 1 for progress, 2 for data header, 3 for data

            for char in range(0,len(list(data))):
                config_header_message.data[char] = ord((list(data))[char])

            self.marv_sys_ctrl_scenario_config_publisher_.publish(config_header_message)

    def send_notification(self,data):
        if not len(list(data)) > self.notification_message_length:
            # Form character list with a fixed length
            notification_string = list(data)
            notification_string.extend([" "] * (self.notification_message_length - len(list(data))))

            #print("String: " + str(notification_string))
            #print("Length: " + str(len(notification_string)))

            # Pack list into seperate messages containing and id and 7 data characters, then publish to can bridge
            for i in range(0,int(self.notification_message_length/7)):
                message = Notification()
                message.location = i
                for j in range(0,7):
                    message.data[j] = ord(notification_string[i*7+j])

                self.marv_sys_ctrl_notification_message_publisher_.publish(message)
                time.sleep(0.001) # Prevent PDU CAN from crashing
                #print("Loc: " + str(message.location))
                #print("Data: " + str(message.data))
        
    def send_cmdSteering(self,aps,rps,angle):
        cmdSteering_message = CmdSteering()
        cmdSteering_message.aps = aps
        cmdSteering_message.rps = rps
        cmdSteering_message.angle = angle
        self.marv_sys_ctrl_cmdSteering_publisher_.publish(cmdSteering_message)

    # Resets the state handler node
    def reset_scenario_handler(self):
        # Set state
        self.scenario_handler_state = self.handler_state["STOPPED"]

        # Set cmdSteering output to 0
        self.send_cmdSteering(0.0, 0.0, 0.0)

        # reset cmdSteering variables
        self.steering_cmd_aps = 0.0
        self.steering_cmd_rps = 0.0
        self.steering_cmd_angle = 0.0

        # Destroy all node publishes and subscriptions
        if not self.node_subscriptions_destroyed:
            self.node_subscriptions_destroyed = True

            self.destroy_publisher(self.node_scenario_start_request_publisher_)
            self.destroy_subscription(self.node_state_subscription)
            self.destroy_subscription(self.node_cmdSteering_subscription)
            self.destroy_subscription(self.node_progress_update_subscription)
            self.destroy_subscription(self.node_data_1_update_subscription)
            self.destroy_subscription(self.node_data_2_update_subscription)
            self.destroy_subscription(self.node_send_notification_subscription)
            self.destroy_subscription(self.node_send_buzzer_signal_subscription)

        # Set active scenario to none
        self.active_scenario = -1

        # Set scenario to not running
        self.scenario_running = False

        # Stop logging
        logging_message = Bool()
        logging_message.data = False
        self.marv_sys_ctrl_log_data_publisher_.publish(logging_message)

        #print("Scenario handler resetted")
        self.get_logger().info("Scenario handler resetted")

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    marv_scenario_handler = MARV_Scenario_Handler()

    rclpy.spin(marv_scenario_handler)

    marv_scenario_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()