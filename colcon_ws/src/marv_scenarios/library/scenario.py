#!/usr/bin/env python3
''' 
# ------------------------------------- #
# Scenario (Wrapper)                    #
# By Viktor Lindstrom, Spring 2021      #
# Chalmers University of Technology     #
# ------------------------------------- #
# The Scenario node handles the logic for starting and stopping scenarios.
# Should be inherited by the "user scenario" to provide the nessecaery
# methods for controlling the waverunner. Only the Set, Get and Update functions
# should be used! Do not directly modify any variables.
'''

# System imports
import time

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Messages
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String
from waverunner_msgs.msg import CmdSteering
from waverunner_msgs.msg import DataVarHeader
from geometry_msgs.msg import PoseWithCovariance
class Scenario(Node):

    ##################################### INIT #############################################################
    def __init__(self,temp,scenario_name,data_var_1_header,data_var_2_header):
        # Setup parameters, variables, publishers, subscribers and timers. Used for setting up the basic funcitons of the scenario wrapper
        super().__init__("scenario_" + temp)

        self.__scenario_name = scenario_name

        self.get_logger().info("Initializing Scenario Wrapper...")

        # Configuration parameters 
        self.__node_state_publisher_update_period = 0.01 # seconds, sends the state to the scenario handler
        self.__node_cmdSteering_init_update_message_period = 0.1 # seconds update time for initial cmdSteering messages
        self.__node_state_update_period = 0.05 # seconds update time for state transitions
        self.__node_update_data_var_header_message_period = 2.0 # seconds update time for updating the data_var_header variables

        # OCU display variables (progress and custom status variables)
        self.__scenario_progress = "0/0"
        self.__scenario_data_var_1_header = ""
        self.__scenario_data_var_1 = "0"
        self.__scenario_data_var_2_header = ""
        self.__scenario_data_var_2 = "0"
        self.__notification_message = "NOT SET"
        self.__buzzer_signal = 0

        # Scenario node states
        self.__state = {
            "INITIALIZING": 0,
            "EXECUTING":    1,
            "FAULT":        2,
            "STOPPED":      3,
            "FINISHED":     4
        }

        # Control mode states, External or Manual mode
        self.__mode_state = {
            "MAN":  0,
            "EXT":  1
        }

        # Node state
        self.__node_state = self.__state["STOPPED"]

        # Waverunner system mode
        self.__mode = self.__mode_state["MAN"]

        # Steering command variables
        self.__steering_cmd_aps = 0.0
        self.__steering_cmd_rps = 0.0
        self.__steering_cmd_angle = 0.0

        # If the node is running, i.e. the scenario has been started and is ongoing
        self.__scenario_running = False

        ######### SUBSCRIPTIONS #########
        # Create node start request subscriber
        node_start_stop_request_TOPIC = "/waverunner/node/" + self.__scenario_name + "/start_scenario"
        self.subscription = self.create_subscription(Bool, node_start_stop_request_TOPIC, self.__node_start_stop_request_callback, 10)
        self.subscription

        ######### PUBLISHES #############
        # Create data val header publisher
        self.__data_var_header_publisher_ = self.create_publisher(DataVarHeader, "/waverunner/node/data_var_header", 10)      

        # Allow time for subscriptions and publishes to be set up
        time.sleep(0.5)

        # Update data_var_headers
        self.set_data_var_1_header(data_var_1_header)
        self.set_data_var_2_header(data_var_2_header)

        # Set a timer for publishing the node state, needs to be done often since the scenario handler otherwise counts the node as not connected
        self.__node_state_publisher_timer = self.create_timer(self.__node_state_publisher_update_period, self.__node_state_publisher_callback)
        self.__node_state_publisher_timer.cancel()
        # Set a timer for updating state transitions
        self.__node_state_update_timer = self.create_timer(self.__node_state_update_period,self.__scenario_state_update_callback)
        self.__node_state_update_timer.cancel()
        # Set a timer for updating cmdSteering
        self.__node_cmdSteering_init_update_timer = self.create_timer(self.__node_cmdSteering_init_update_message_period,self.__node_cmdSteering_update_callback)
        self.__node_cmdSteering_init_update_timer.cancel() # Stop timer
        # Set a timer for updating the data_val_header, sending it to the scenario handler
        self.__node_update_data_var_header_timer = self.create_timer(self.__node_update_data_var_header_message_period,self.__node_update_data_var_header_callback)

        self.get_logger().info("Scenario Wrapper Started...")

    ########################################################################################################

    ##################################### SETUP AND TEARDOWN ###############################################

    # Setup all publishes, timer etc. Called to start up the full function of the wrapper when the scenario is starting
    def __setup(self):

        ######### SUBSCRIPTIONS #########
        # Subscribe to the waverunner system mode
        self.mode_subscription = self.create_subscription(Bool, '/waverunner/sys/status/state_12V_auto', self.__mode_callback, 10)
        self.mode_subscription

        ######### PUBLISHES #############
        # Create node state publisher
        node_state_TOPIC = "/waverunner/node/" + self.__scenario_name + "/state"
        self.__node_state_publisher_ = self.create_publisher(Int8, node_state_TOPIC, 10)
        # Create node cmdSteering publisher
        node_cmdSteering_TOPIC = "/waverunner/node/" + self.__scenario_name + "/cmdSteering"
        self.__node_cmdSteering_publisher_ = self.create_publisher(CmdSteering, node_cmdSteering_TOPIC, 10)
        # Create scenario progress publisher
        node_progress_update_TOPIC = "/waverunner/node/" + self.__scenario_name + "/progress_update"
        self.__node_progress_update_publisher_ = self.create_publisher(String, node_progress_update_TOPIC, 10)
        # Create scenario data var 1 publisher
        node_data_1_update_TOPIC = "/waverunner/node/" + self.__scenario_name + "/data_var_1_update"
        self.__node_data_var_1_update_publisher_ = self.create_publisher(String, node_data_1_update_TOPIC, 10)
        # Create scenario data var 1 publisher
        node_data_2_update_TOPIC = "/waverunner/node/" +self.__scenario_name + "/data_var_2_update"
        self.__node_data_var_2_update_publisher_ = self.create_publisher(String, node_data_2_update_TOPIC, 10)
        # Create notification publisher
        node_send_notification_TOPIC = "/waverunner/node/" + self.__scenario_name + "/send_notification"
        self.__node_send_notification_publisher_ = self.create_publisher(String, node_send_notification_TOPIC, 10)
        # Create buzzer signal publisher
        node_send_buzzer_signal_TOPIC = "/waverunner/node/" + self.__scenario_name + "/send_buzzer_signal"
        self.__node_send_buzzer_signal_publisher_ = self.create_publisher(Int8, node_send_buzzer_signal_TOPIC, 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(0.5)

        # Start timers for publishing state and state transition updates
        self.__node_state_publisher_timer.reset()
        self.__node_state_update_timer.reset()

        self.get_logger().info("Scenario Wrapper Running...")

    # Used to destroy publishes, subscriptions and stop timers when the scenario has stopped
    def __tear_down(self):
        
        self.destroy_subscription(self.mode_subscription)
        self.destroy_publisher(self.__node_state_publisher_)
        self.destroy_publisher(self.__node_cmdSteering_publisher_)
        self.destroy_publisher(self.__node_progress_update_publisher_)
        self.destroy_publisher(self.__node_data_var_1_update_publisher_)
        self.destroy_publisher(self.__node_data_var_2_update_publisher_)
        self.destroy_publisher(self.__node_send_notification_publisher_)
        self.destroy_publisher(self.__node_send_buzzer_signal_publisher_)
        
        self.__node_state_publisher_timer.cancel()
        self.__node_state_update_timer.cancel()

        self.get_logger().info("Scenario Wrapper Resetted...")

    ########################################################################################################

    ##################################### NODE HANDLER CALLBACKS ###########################################

    # Perform automatic state transitions
    def __scenario_state_update_callback(self):
        if self.__mode == self.__mode_state["MAN"] and not self.__node_state == self.__state["INITIALIZING"] and not self.__node_state == self.__state["STOPPED"]:
            self.__node_state = self.__state["STOPPED"]
            self.__scenario_running = False
            self.__tear_down()
            print("State: STOPPED")

        elif self.__mode == self.__mode_state["EXT"] and self.__node_state == self.__state["INITIALIZING"]:
            self.__node_state = self.__state["EXECUTING"]
            self.__scenario_running = True
            self.__node_cmdSteering_init_update_timer.cancel()
            print("State: EXECUTING")

    # Handles a start and stop request from the scenario handler
    def __node_start_stop_request_callback(self,ros_msg):
        if ros_msg.data == True:
            self.__setup()
            self.__node_state = self.__state["INITIALIZING"]
            self.__node_cmdSteering_init_update_timer.reset()
            print("State: INITIALIZING")
        else:
            self.__node_state = self.__state["STOPPED"]
            self.__node_cmdSteering_init_update_timer.cancel()
            self.__scenario_running = False
            self.__tear_down()
            print("State: STOPPED")

    # Mode callback, listens to the system 12V auto, activated => External mode, deactivated => Manual mode
    def __mode_callback(self,ros_msg):
        if ros_msg.data == False:
            self.__mode = self.__mode_state["MAN"]
        else:
            self.__mode = self.__mode_state["EXT"]

    # Publishes the node state 
    def __node_state_publisher_callback(self):
        node_state_message = Int8()
        node_state_message.data = self.__node_state
        self.__node_state_publisher_.publish(node_state_message)

    # Publish steering commands to the scenario handler
    def __node_cmdSteering_update_callback(self):
        self.__steering_cmd_aps = 0.0
        self.__steering_cmd_rps = 0.0
        self.__steering_cmd_angle = 0.0
        self.__node_cmdSteering_update(self.__steering_cmd_aps,self.__steering_cmd_rps,self.__steering_cmd_angle)

    # Publish the data val headers set in the scenario
    def __node_update_data_var_header_callback(self):
        data_var_header_message = DataVarHeader()
        data_var_header_message.header = self.__scenario_name
        data_var_header_message.data_1_header = self.__scenario_data_var_1_header
        data_var_header_message.data_2_header = self.__scenario_data_var_2_header
        self.__data_var_header_publisher_.publish(data_var_header_message)

    ########################################################################################################

    ##################################### GETTERS ##########################################################

    def get_scenario_running(self):
        '''Returns a bool if the scenario is running or not.
        Returns: Bool
        - False = Not Running 
        - True = Running
        '''

        return self.__scenario_running

    def get_scenario_state(self):
        '''Returns the state of the scenario.
        Returns: String
        - INITIALIZING
        - EXECUTING
        - FAULT
        - STOPPED
        - FINISHED
        '''

        inverted_state = dict(map(reversed,self.__state.items()))
        return inverted_state[self.__node_state]

    def get_cmd_steering_angle(self):
        '''Returns the set cmd steering angle.
        Returns: Float
        '''

        return self.__steering_cmd_angle

    def get_cmd_steering_aps(self):
        '''Returns the set cmd aps value (throttle, forwards).
        Returns: Float
        '''

        return self.__steering_cmd_aps

    def get_cmd_steering_rps(self):
        '''Returns the set cmd rps value (throttle, backwards).
        Returns: Float
        '''

        return self.__steering_cmd_rps

    def get_progress(self):
        '''Returns the set progress variable that is shown on the OCU display.
        Returns: String
        '''

        return self.__scenario_progress

    def get_data_var_1(self):
        '''Returns the set data_var_1 variable that is shown on the OCU display.
        Returns: String
        '''

        return self.__scenario_data_var_1

    def get_data_var_2(self):
        '''Returns the set set data_var_2 variable that is shown on the OCU display.
        Returns: String
        '''
        
        return self.__scenario_data_var_2

    def get_data_var_1_header(self):
        '''Returns the data var 1 header that is shown on the OCU display
        Returns: String
        '''

        return self.__scenario_data_var_1_header

    def get_data_var_2_header(self):
        '''Returns the data var 1 header that is shown on the OCU display
        Returns: String
        '''

        return self.__scenario_data_var_2_header

    ########################################################################################################

    ##################################### SETTERS ##########################################################

    def set_state_finished(self):
        '''Sets the the state of the scenario to FINISHED,
            indicating that the scenario has finished.
        Returns: Bool
        - False = State not set, scenario not running
        - True = State set
        '''

        if self.__scenario_running:
            self.__node_state = self.__state["FINISHED"]
            self.__scenario_running = False
            return True
        else:
            return False

    def set_state_fault(self):
        '''Sets the the state of the scenario to FAULT, 
            indicating and error that should stop the scenario.
        Returns: Bool
        - False = State not set, scenario not running
        - True = State set
        '''

        if self.__scenario_running:
            self.__node_state = self.__state["FAULT"]
            self.__scenario_running = False
            return True
        else:
            return False

    def set_cmd_steering_angle(self,angle):
        '''Sets the cmd steering angle.
        Returns: Bool
        - False = Not set, outside of allowd input interval
        - True = Set
        Parameter:
        - angle (Float) = The angle of the handle
        Input Range:
        - [-22,22] deg
        '''

        if angle >= -22.0 and angle <= 22.0:
            self.__steering_cmd_angle = float(angle)
            return True
        else:
            return False

    def set_cmd_steering_aps(self,aps):
        '''Sets the cmd steering aps value (throttle, forwards). 
            The aps and rps values are exclusive, therefore only one can be set at
            a time. This function therefore sets rps to 0.0
        Returns:
        - False = Not set, outside of allowd input interval
        - True = Set
        Parameter:
        - aps (Float) = The aps throttle value
        Input Range:
        - [0,100] percent
        '''

        if aps >= 0.0 and aps <= 100.0:
            self.__steering_cmd_aps = float(aps)
            self.__steering_cmd_rps = 0.0
            return True
        else:
            return False

    def set_cmd_steering_rps(self,rps):
        '''Sets the cmd steering rps value (throttle, backwards). 
            The aps and rps values are exclusive, therefore only one can be set at
            a time. This function therefore sets aps to 0.0
        Returns:
        - False = Not set, outside of allowd input interval
        - True = Set
        Parameter:
        - rps (Float) = The aps throttle value
        Input Range:
        - [0,100] percent
        '''

        if rps >= 0.0 and rps <= 100.0:
            self.__steering_cmd_rps = float(rps)
            self.__steering_cmd_aps = 0.0
            return True
        else:
            return False

    def set_progress(self,data):
        '''Sets the the progress variable to display on the OCU, max 7 characters.
        Returns: Bool
        - False = Variable not set, input string longer than 7 characters
        - True = Variable set
        Parameter:
        - data (String) = The progress string
        '''

        if not len(list(data)) > 7:
            self.__scenario_progress = data
            return True
        elif data == None or data == "":
            self.__scenario_progress = "0/0"
        else:
            return False

    def set_data_var_1(self,data):
        '''Sets the the data_var_1 variable to display on the OCU, max 7 characters.
        Returns: Bool
        - False = Variable not set, input string longer than 7 characters
        - True = Variable set
        Parameter:
        - data (String) = The data_var_1 string
        '''

        if not len(list(data)) > 7:
            self.__scenario_data_var_1 = data
            return True
        elif data == None or data == "":
            self.__scenario_data_var_1 = "0"
        else:
            return False

    def set_data_var_2(self,data):
        '''Sets the the data_var_2 variable to display on the OCU, max 7 characters.
        Returns: Bool
        - False = Variable not set, input string longer than 7 characters
        - True = Variable set
        Parameter:
        - data (String) = The data_var_2 string
        '''

        if not len(list(data)) > 7:
            self.__scenario_data_var_2 = data
            return True
        elif data == None or data == "":
            self.__scenario_data_var_2 = "0"
        else:
            return False

    def set_data_var_1_header(self,data):
        '''Sets the data_var_1_header to display on the OCU, max 7 charachters.
        Returns: BOOL
        - False = Variable not set, input string longer than 7 characters
        - True = Variable set
        Parameter:
        - data (String) = The data_var_1_header string
        '''

        if not len(list(data)) > 7:
            self.__scenario_data_var_1_header = data
            return True
        elif data == None or data == "":
            self.__scenario_data_var_1_header = ""
        else:
            return False

    def set_data_var_2_header(self,data):
        '''Sets the data_var_2_header to display on the OCU, max 7 charachters.
        Returns: BOOL
        - False = Variable not set, input string longer than 7 characters
        - True = Variable set
        Parameter:
        - data (String) = The data_var_2_header string
        '''

        if not len(list(data)) > 7:
            self.__scenario_data_var_2_header = data
            return True
        elif data == None or data == "":
            self.__scenario_data_var_2_header = ""
        else:
            return False

    def set_notification_message(self,data):
        '''Sets the the notification message to display as a popup on the OCU, max 70 characters.
            String needs to use "#" as delimiters for new rows. It also needs to end with a delimiter,
            example: myString = "THIS IS#AN#EXAMPLE#STRING# #WITH ONE#BLANK#ROW#"
        Returns: Bool
        - False = Message not set, input string longer than 70 characters
        - True = Message set
        Parameter:
        - data (String) = The notification message string
        '''

        if not len(list(data)) > 70:
            self.__notification_message = data
            return True
        elif data == None or data == "":
            self.__notification_message = "NOT SET#"
        else:
            return False

    def set_buzzer_signal(self,data):
        '''Sets the buzzer signal output to make the buzzer sound.
        Returns: Bool
        - True = Message set
        Parameter:
        - data (int8) = The sound signal ID
        '''

        self.__buzzer_signal = int(data)
        return True

    ########################################################################################################

    ##################################### UPDATES ##########################################################

    def update_cmd_steering(self):
        '''Updates the cmd steering output, publishing the values to the system.
        Returns: Bool
        - False = Not updated, scenario not running
        - True = Updated
        '''

        if self.__scenario_running:
            self.__node_cmdSteering_update(self.__steering_cmd_aps,self.__steering_cmd_rps,self.__steering_cmd_angle)
            return True
        else:
            return False

    def update_progress(self):
        '''Updates the progress variable output, publishing the values to the system.
        Returns: Bool
        - False = Not updated, scenario not running
        - True = Updated
        '''

        if self.__scenario_running:
            progress_message = String()
            progress_message.data = str(self.__scenario_progress)
            self.__node_progress_update_publisher_.publish(progress_message)
            return True
        else:
            return False

    def update_data_var_1(self):
        '''Updates the data_var_1 variable output, publishing the values to the system.
        Returns: Bool
        - False = Not updated, scenario not running
        - True = Updated
        '''

        if self.__scenario_running:
            data_var_1_message = String()
            data_var_1_message.data = str(self.__scenario_data_var_1)
            self.__node_data_var_1_update_publisher_.publish(data_var_1_message)
            return True
        else:
            return False

    def update_data_var_2(self):
        '''Updates the data_var_2 variable output, publishing the values to the system.
        Returns: Bool
        - False = Not updated, scenario not running
        - True = Updated
        '''

        if self.__scenario_running:
            data_var_2_message = String()
            data_var_2_message.data = str(self.__scenario_data_var_2)
            self.__node_data_var_2_update_publisher_.publish(data_var_2_message)
            return True
        else:
            return False

    def update_all_variables(self):
        '''Updates progress, data_var_1 and data_var2 variables outputs, publishing 
            all the values at once to the system.
        Returns: Bool
        - False = Not updated, scenario not running
        - True = Updated
        '''

        if self.__scenario_running:
            self.update_progress()
            self.update_data_var_1()
            self.update_data_var_2()
            return True
        else:
            return False

    def update_notification_message(self):
        '''Updates the notification message output, publishing the message to the system
            causing a popup mesage to show on the OCU.
        Returns: Bool
        - False = Not updated, scenario not running
        - True = Updated
        '''

        if self.__scenario_running:
            notification_message = String()
            notification_message = str(self.__notification_message)
            self.__node_send_notification_publisher_.publish(notification_message)
            return True
        else:
            return False

    def update_buzzer_signal(self):
        '''Updates the buzzer signal output, publishing the message to the system
            causing the buzzer to sound.
        Returns: Bool
        - False = Not updated, scenario not running
        - True = Updated
        '''

        if self.__scenario_running:
            buzzer_signal_message = Int8()
            buzzer_signal_message.data = int(self.__buzzer_signal)
            self.__node_send_buzzer_signal_publisher_.publish(buzzer_signal_message)
            return True
        else:
            return False

    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################

    # Publish steering commands to the scenario handler
    def __node_cmdSteering_update(self,steering_cmd_aps,steering_cmd_rps,steering_cmd_angle):
        node_cmdSteering_message = CmdSteering()
        node_cmdSteering_message.aps = float(steering_cmd_aps)
        node_cmdSteering_message.rps = float(steering_cmd_rps)
        node_cmdSteering_message.angle = float(steering_cmd_angle)
        
        self.__node_cmdSteering_publisher_.publish(node_cmdSteering_message)

    ########################################################################################################



