#!/usr/bin/env python3

# --------------------------------------- #
# WaveRunner Waypoint Following Algorithm #
# By Viktor Lindstrom and                 #
# Noel Danielsson, Summer 2021            #
# Chalmers University of Technology       #
# --------------------------------------- #

import numpy as np

class Waypoint_Algorithm:
    def __init__(self,waypoints,parameters):
        '''
        waypoints: numpy matrix Nx4, (wp_eta_x, wp_eta_y, wp_vel_seg, wp_radius)
            wp_eta_x = x coordinates of waypoints (m)
            wp_eta_y = y coordinates of waypoints (m)
            wp_vel_seg = velocities of the bridge/segment leading up to the waypoint (m/s)
            wp_radius = the radius in which the waypoint counts as taken (m)
        parameters: numpy array 1x4, (u_min, delta_t, surge_rate, vel_diff_ang)
            u_min = minimum velocity (in order to retain the possibility to steer correctly) (m/s)
            delta_t = the measured time between reference and system response when chaning surge (s)
            surge_rate = maximum acceleration/deceleration (m/s^2)
            vel_diff_ang = within this tolerance angle, the maximum velocity at the current bridge will be used, outside the velocity will be lowered (deg)
        '''

        # Store all waypoints
        #self.waypoints = waypoints
        self.waypoints = waypoints
        self.current_wp = 0 # index
        if self.waypoints.ndim == 1:
            self.wp_eta = self.waypoints[0:2]
            self.wp_vel_seg = self.waypoints[2]
            self.wp_radius = self.waypoints[3]
        else:
            self.wp_eta = self.waypoints[self.current_wp,0:2]
            self.wp_vel_seg = self.waypoints[self.current_wp,2]
            self.wp_radius = self.waypoints[self.current_wp,3]

        # Unpack parameters
        self.u_min = parameters[0]
        self.delta_t = parameters[1]
        self.surge_rate = parameters[2]
        self.vel_diff_ang = parameters[3]

        # Reference variables
        self.vel_ref = 0 # Velocity reference (m/s)
        self.head_ref = 0 # Heading reference (rad)

        # Other variables
        self.start_position = np.array([0, 0]) # Corrdinates from which the first bridge will be calculated from
        self.finished = False
        self.inside_wp = False

    def update(self,eta):
        '''
        eta: numpy array 1x2, (eta_x, eta_y, eta_psi)
            eta_x = current x position (m)
            eta_y = current y position (m)
            eta_psi = current heading (rad)
        '''

        # Unpack current waypoint
        if self.waypoints.ndim == 1:
            self.wp_eta = self.waypoints[0:2]
            self.wp_vel_seg = self.waypoints[2]
            self.wp_radius = self.waypoints[3]
        else:
            self.wp_eta = self.waypoints[self.current_wp,0:2]
            self.wp_vel_seg = self.waypoints[self.current_wp,2]
            self.wp_radius = self.waypoints[self.current_wp,3]

        # Set start position after first waypoint
        if self.current_wp > 0:
            self.start_position = self.waypoints[self.current_wp-1,0:2]

        # vector from WR to WP
        wp_heading = self.wp_eta - eta[0:2]

        # LOS toward WP
        self.head_ref = np.arctan2(wp_heading[1],wp_heading[0])

        # Check if finished, i.e. reached the last waypoint
        if (self.current_wp+1 >=  np.size(self.waypoints,0) and np.linalg.norm(wp_heading) < self.wp_radius) or self.finished:
            self.finished = True
            self.head_ref = eta[2]
            self.vel_ref = 0
        
        # If there are waypoints remaning
        elif self.current_wp+1 < np.size(self.waypoints,0) and self.waypoints.ndim != 1:
            
            # Calculate bridge angle between current and next wp
            wp_eta_next = self.waypoints[self.current_wp+1,0:2]
            bridge = self.wp_eta - self.start_position
            bridge_next = wp_eta_next - self.wp_eta
            bridge_angle = np.arccos(np.dot(bridge,bridge_next)/(np.linalg.norm(bridge)*np.linalg.norm(bridge_next)))

            # Calculate the radius of the velocity change circle
            vel_change = np.sin(0.5*bridge_angle) * (self.wp_vel_seg - self.u_min)
            vel_change_radius = 0
            if vel_change == 0: # If no angle difference
                vel_change_radius = self.wp_radius
            else:
                vel_change_radius = self.wp_radius + self.delta_t * self.wp_vel_seg + 0.5*(np.power(vel_change,2))/self.surge_rate
                
            
            if np.linalg.norm(wp_heading) < vel_change_radius: # Slow down if inside cirle
                self.vel_ref = self.wp_vel_seg - vel_change
            else: # Otherwise, calculate the velocity reference, lower velocity if outside vel_diff_ang tolerance
                angle_diff = self.head_ref - eta[2]
                if angle_diff > np.pi:
                    angle_diff = angle_diff - 2*np.pi
                elif angle_diff <= - np.pi:
                    angle_diff = angle_diff + 2*np.pi
                angle_diff = np.absolute(angle_diff)

                if angle_diff > np.deg2rad(self.vel_diff_ang):
                    vel_change_angle = np.sin(0.5*angle_diff - np.deg2rad(self.vel_diff_ang)) * (self.wp_vel_seg - self.u_min)
                    self.vel_ref = self.wp_vel_seg - vel_change_angle
                else:
                    self.vel_ref = self.wp_vel_seg

            # If inside the waypoint radius
            if np.linalg.norm(wp_heading) < self.wp_radius:
                self.inside_wp = True
                # Set heading towards next wp
                wp_heading_next = wp_eta_next - eta[0:2]
                # LOS towards next wp
                self.head_ref = np.arctan2(wp_heading_next[1],wp_heading_next[0])

            # If going out of wp_radius, change to next wp
            if np.linalg.norm(wp_heading) >= self.wp_radius and self.inside_wp:
                self.inside_wp = False
                self.current_wp += 1
                # Set heading towards the next wp
                wp_heading_next = wp_eta_next - eta[0:2]
                self.head_ref = np.arctan2(wp_heading_next[1],wp_heading_next[0])

        else: # On last waypoint
            angle_diff = self.head_ref - eta[2]
            if angle_diff > np.pi:
                angle_diff = angle_diff - 2*np.pi
            elif angle_diff <= - np.pi:
                angle_diff = angle_diff + 2*np.pi
            angle_diff = np.absolute(angle_diff)

            if angle_diff > np.deg2rad(self.vel_diff_ang):
                vel_change_angle = np.sin(0.5*angle_diff - np.deg2rad(self.vel_diff_ang)) * (self.wp_vel_seg - self.u_min)
                self.vel_ref = self.wp_vel_seg - vel_change_angle
            else:
                self.vel_ref = self.wp_vel_seg

    def get_vel_ref(self):
        return self.vel_ref

    def get_head_ref(self):
        return self.head_ref

    def get_finished_state(self):
        if self.finished:
            return True
        else:
            return False

    def get_current_wp(self):
        return self.current_wp

    def get_nbr_of_wp(self):
        if self.waypoints.ndim == 1:
            return 1
        else:
            return np.size(self.waypoints,0)

    def get_wp_eta(self):
        return self.wp_eta

    def get_wp_vel_seg(self):
        return self.wp_vel_seg

    def set_start_position(self,start_position):
        '''
        start_position: numpy array 1x2, (x_start, y_start)
            x_start = x start position (m)
            y_start = y start position (m)
        '''
        self.start_position = start_position

    def set_current_wp(self,new_current_wp):
        self.current_wp = new_current_wp

    def set_finished_state(self,state):
        self.finished = state
