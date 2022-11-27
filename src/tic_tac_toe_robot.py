#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
# Modified by Keara Berlin (kearaberlin@gmail.com) Nov 2022
#
# Sets up robot and provides methods to mark a square on a tic tac toe board centered in front of the robot.
# 	mark_square(n) Marks square n on the board with a dot. 
#
###

import sys
import rospy
import time
import math
import random
from kortex_driver.srv import *
from kortex_driver.msg import *

class TicTacToeRobot:

    #RESTING_POSE = [0, 0.35, 0.08, -90, 0, 0]
    RESTING_POSE = [0.1, -0.2, 0.08, 90, 0, 90]
    CENTER_POSE = [0.4, -0.4, 0.08, 90, 0, 90]
        
    def __init__(self):
        try:
            rospy.init_node('tic_tac_toe')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")

            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True

    # move end effector to cartesian pose [x, y, z, theta_x, theta_y, theta_z] in meters and degrees
    def go_to_pose(self, pose):
        n = random.randrange(100)
        
        self.example_clear_faults()
    
        my_cartesian_speed = CartesianSpeed()
        my_cartesian_speed.translation = 0.1 # m/s
        my_cartesian_speed.orientation = 15  # deg/s

        my_constrained_pose = ConstrainedPose()
        my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
        
        my_constrained_pose.target_pose.x = pose[0]
        my_constrained_pose.target_pose.y = pose[1]
        my_constrained_pose.target_pose.z = pose[2]
        my_constrained_pose.target_pose.theta_x = pose[3]
        my_constrained_pose.target_pose.theta_y = pose[4]
        my_constrained_pose.target_pose.theta_z = pose[5]

        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
        req.input.name = f"pose{n}"
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = 1001 + n

        rospy.loginfo("Sending pose...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose")
            success = False
        else:
            rospy.loginfo("Waiting for pose to finish...")

        self.wait_for_action_end_or_abort()

    # Mark square n with a dot.
    # 0 1 2
    # 3 4 5
    # 6 7 8
    def mark_square(self, n):
        dz = 0.03
        square_width = 0.05
        orthog_dist = 1/math.sqrt(2) * square_width
        
        print(f"Orthog dist: {orthog_dist}")
        
        # center square coordinates
        pose = TicTacToeRobot.CENTER_POSE
        x = pose[0]	# 0
        y = pose[1] 	# 0.55
        z = pose[2]	# 0.08
        # orientation
        tx = pose[3] 	# -90
        ty = pose[4]	# 0
        tz = pose[5]	# 0
        
        ## find coords if not marking center square
        # column
        if n % 3 == 0: # go left (+x, +y)
            x += orthog_dist
            y += orthog_dist
        elif (n-2) % 3 == 0: # go right (-x, -y)
            x += -orthog_dist
            y += -orthog_dist
        #row
        if n <= 2: # go up (+x, -y)
            x += orthog_dist
            y += -orthog_dist
        elif n >= 6: # go down (-x, +y)
            x += -orthog_dist
            y += orthog_dist
            
        # move to pose
        self.go_to_pose([x, y, z, tx, ty, tz])
        
        # move down by dz to touch paper
        self.go_to_pose([x, y, z-dz, tx, ty, tz])
        
        # move back up
        self.go_to_pose([x, y, z, tx, ty, tz])
        
        self.go_to_pose(TicTacToeRobot.RESTING_POSE)
        

    def main(self):
        success = self.is_init_success

        if success:

            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Start from resting position
            # needs to go along a certain path or it may encounter errors
            success &= self.example_home_the_robot()
            self.go_to_pose([0.6, 0, 0.2, 90, 0, 90])
            self.go_to_pose([0.5, -0.2, 0.2, 90, 0, 90])
            self.go_to_pose([0.5, -0.2, 0.08, 90, 0, 90])
            self.go_to_pose(TicTacToeRobot.RESTING_POSE)
            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            #*******************************************************************************
            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.example_subscribe_to_a_robot_notification()
            
            rospy.spin()


if __name__ == "__main__":
    ex = TicTacToeRobot()
    ex.main()
