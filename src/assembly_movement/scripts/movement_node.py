#!/usr/bin/env python

import argparse
import rospy
import numpy as np
import os
import random
import math
import struct

# sourcing from local ROS environment 
from assembly_movement.msg import (
    Position,
)
from assembly_movement.srv import (
    ToPosition,
)
from std_msgs.msg import (
    UInt16,
    Header,
)
from netft_utils.srv import (
    SetBias,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
    WrenchStamped,
)
import baxter_interface
from baxter_interface import CHECK_VERSION

def namCheckandReturn(text,i,j,arr,suf, pre):
    if(text[i+j][-2:] == suf and text[i+j][:3] == pre):
        name = text[i+j]
        value = text[i+j+2]
        if(arr == None):
            arr = np.array([name,value])
        else:
            arr = np.append(arr,[name,value])

    return(arr)

def capturePosOri(words, i, arr):
    # gets the position number for sub array
    arr.append(words[i].split("|")[1])

    # iterated through the found sub array
    for j in range(21):
        # goes through and collects the pisition name and value
        arr = namCheckandReturn(words,i,j,arr,".x","pos")
        arr = namCheckandReturn(words,i,j,arr,".y","pos")
        arr = namCheckandReturn(words,i,j,arr,".z","pos")
        arr = namCheckandReturn(words,i,j,arr,".x","ori")
        arr = namCheckandReturn(words,i,j,arr,".y","ori")
        arr = namCheckandReturn(words,i,j,arr,".z","ori")
        arr = namCheckandReturn(words,i,j,arr,".w","ori")

    return(arr)

def read(file):
    f = open(file, "r")
    words =  f.read().split()

    positions = []

    for i, word in enumerate(words):
        hold = []

        # searches the text file for position phrase
        if(word[:9] == 'position_'):
            # amends the required information from the found sub array
            positions.append(capturePosOri(words, i, hold))
    # orgnizes the new filled array based on position number
    positions = sorted(positions, key=lambda x:x[0])
    f.close()

    return positions

class Movement(object):
    def __init__(self, file):
        """
        Intial the Object with all the required setting for Baxter
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',UInt16, queue_size=10)

        # setting up Baxter limb control
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_arm = baxter_interface.limb.Limb("left")
        self.current_right_arm = self._right_arm.endpoint_pose()
        self.current_left_arm = self._left_arm.endpoint_pose()
        self._left_arm.set_joint_position_speed(0.3)
        self._right_arm.set_joint_position_speed(0.3)
        self._left_joint_names = self._left_arm.joint_names()

        # getting the position text file
        self.pos = file

        # Sets force limit flag to false
        self.max_wrench = False

        # Sets defult system rate
        self._rate = 500.0  # Hz

        # enables Baxter on start up of script
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._position_service = rospy.Service('%s/Position'%rospy.get_name(), ToPosition, self.position_cmd)

        # List where saving the required actions
        self.actions_list = []

        rospy.loginfo('%s::Baxter: Enabling robot...'%(rospy.get_name()))
        self._rs.enable()
    
        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

    def reset_control_modes(self):
        """
        Returns the system back to a normal state on restart and shutdown
        """
        rate = rospy.Rate(self._rate)
        for _ in np.arange(100):
            if rospy.is_shutdown():
                return False
            self._right_arm.exit_control_mode()
            self._left_arm.exit_control_mode()
            self._pub_rate.publish(100)
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        rospy.loginfo('%s::Baxter: Moving to neutral pose...'%(rospy.get_name()))
        self._right_arm.move_to_neutral()
        self._left_arm.move_to_neutral()
        if not rospy.is_shutdown():
            rospy.loginfo('%s::Baxter: Ready...\n'%(rospy.get_name()))

    def clean_shutdown(self):
        """
        Configure the action to be done on shutdown 
        """
        rospy.loginfo('\n%s::Baxter: Exiting...'%(rospy.get_name()))

        self.reset_control_modes()
        if not self._init_state:
            rospy.loginfo('%s::Baxter: Disabling robot...'%(rospy.get_name()))
            self._rs.disable()
        return True

    def ik_request(self,limb,pose):
        """
        IK joint angle solutions for a given pose, as list
        Sourced from SDK Wiki
        """
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            rospy.wait_for_service(ns,5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("**Service call failed: %s**" % (e,))
            return False

        # Check if result valid, and type of seed ultimately used to get solution
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if resp_seeds[0] != resp.RESULT_INVALID:
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            rospy.logerr("**INVALID POSE - No Valid Joint Solution Found**")
            return False
        return limb_joints
    
    def move_to_coords(self, armName, coords):
        """
        Geting the required position coordinates from the selected position
        """
        # create the struct that appropriate for coordantes 
        arm = Pose()
        arm.position.x = float(coords[2])
        arm.position.y = float(coords[4])
        arm.position.z = float(coords[6])
        arm.orientation.x = float(coords[8])
        arm.orientation.y = float(coords[10])
        arm.orientation.z = float(coords[12])
        arm.orientation.w = float(coords[14])

        # checks that ROS core is still running 
        if rospy.is_shutdown():
            return False
        
        # get the joint angles for the IK for the specific limb
        joint_angles = self.ik_request(armName,arm)

        # checks limb name and passes it to the correct joints
        if joint_angles:
            if(armName == "left"):
                self._left_arm.move_to_joint_positions(joint_angles)
            elif(armName == "right"):
                self._right_arm.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("**No Joint Angles Provided**")

    def set_wrench_bias(self):
        """
        resets the ATI force sesnor to a 0 force bias and sets the 
        warning limits 50 for force and 10 for torque
        """
        rospy.wait_for_service('/bias')
        try:
            bias_cmd = rospy.ServiceProxy('/bias', SetBias)
            resp1 = bias_cmd(True,50.0,10.0)
            return resp1
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)


    def wrench_current(self):
        """
        Current wrench values from ATI force-torque sensor. 
        '/transformed_world' is the biased topic
        source from Mats Nilsson
        """

        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/transformed_world", WrenchStamped)
            fx = msg.wrench.force.x
            fy = msg.wrench.force.y
            fz = msg.wrench.force.z
            tx = msg.wrench.torque.x
            ty = msg.wrench.torque.y
            tz = msg.wrench.torque.z
            wrench = np.array([[fx,fy,fz,tx,ty,tz]]).T
            if fz <= -5:
                self.max_wrench = True
            break
        
        return wrench

    def hybrid_movement(self):
        """
        performs the nesercy movement required for hybrid_movement.
        this movement includes moveing the s0 joint while the force 
        torque sensor was lower then set limit.
        """
        rate = rospy.Rate(self._rate)
        start = rospy.Time.now()

        def make_v_func():
            """
            returns a randomly period and speed factor for the s0 joint
            to perform while searhcing for block connection 
            """
            period_factor = random.uniform(0.1, 0.2)
            amplitude_factor = random.uniform(0.005, 0.01)

            # These factors are then put on to a cosine funcaition 
            # so that it is a funcaiton of time to provide smooth
            # moition for the manipulator 
            def v_func(elapsed):
                w = period_factor * elapsed.to_sec()
                return amplitude_factor * math.cos(w * 2 * math.pi)
            return v_func

        jointangle = self._left_arm.joint_angles()
        for name in self._left_joint_names:
            # only provide the s0 joint with this new movement
            if name[-2:] == "s0":
                jointangle[name] = make_v_func()

        # sending the joints the required command 
        def make_cmd(joint_names, elapsed):
            new_cmd = []
            for name in joint_names:
                if name[-2:] == "s0":
                    new_cmd.append((name, jointangle[name](elapsed)))
            return dict(new_cmd)

        # loops unit wrench limit is detected 
        while(self.max_wrench != True):
            self._pub_rate.publish(self._rate)
            elapsed = rospy.Time.now() - start
            cmd = make_cmd(self._left_joint_names, elapsed)
            self._left_arm.set_joint_velocities(cmd)
            rate.sleep()
        print("FIN")
        self.max_wrench = False
        

    def adaptive_coords(self, armName, coords, current):
        """
        moves one gripper to the current position of the other gripper
        """
        arm = Pose()
        # current is the other gripper with some offset
        arm.position.x = float(current[0])
        arm.position.y = float(current[1]-0.395)
        arm.position.z = float(current[2]+0.025)
        arm.orientation.x = float(coords[8])
        arm.orientation.y = float(coords[10])
        arm.orientation.z = float(coords[12])
        arm.orientation.w = float(coords[14])

        if rospy.is_shutdown():
            return False
                
        joint_angles = self.ik_request(armName,arm)
        if joint_angles:
            if(armName == "left"):
                # reduces the speed of the manipulator for better pathing
                self._left_arm.set_joint_position_speed(0.1)
                self._left_arm.move_to_joint_positions(joint_angles)
                # returns the speed rejestor back to normal
                self._left_arm.set_joint_position_speed(0.3)
            elif(armName == "right"):
                self._right_arm.set_joint_position_speed(0.1)
                self._right_arm.move_to_joint_positions(joint_angles)
                self._right_arm.set_joint_position_speed(0.3)

        else:
            rospy.logerr("**No Joint Angles Provided**")

    def position_cmd(self, req):
        """
        collects the position from the ROS service being called
        """
        # the position is then added to the list of positions
        self.actions_list.append(req.Position)
        rospy.loginfo('%s::Position Cmd: Received new Position %s'%(rospy.get_name(), req.Position))
        
        return True
 
    def movement(self):
        # moves limbs to neutral position on startup
        self.set_neutral()
        rate = rospy.Rate(self._rate)

        # control loop only stops when ROS core shutsdown
        while not rospy.is_shutdown():
            # waits for posiiton to be added to the action list
            if len(self.actions_list) > 0:
                rospy.loginfo('%s::Baxter: Current Action List  %s'%(rospy.get_name(), self.actions_list))
                # earliest action is then collected and removed from the list
                action = self.actions_list[0]
                self.actions_list.remove(action)

                self._pub_rate.publish(self._rate)
                
                # the positions are then read from the text file
                pos = read(self.pos)

                # the earliest action is then compaired to the vaild list of actions
                if(action == Position.L_OVER_POS):
                    # once a vaild action is forn the position and its limb is sent 
                    self.move_to_coords('left',pos[0]) #ROS Serive Position number: 1
                elif(action == Position.R_OVER_POS):
                    self.move_to_coords('right',pos[1]) #2
                elif(action == Position.R_HORIZ_POS):
                    self.move_to_coords('right',pos[2]) #3
                elif(action == Position.L_HORIZ_POS):
                    self.move_to_coords('left',pos[3]) #4
                elif(action == Position.R_PICKUP_POS):
                    self.move_to_coords('right',pos[4]) #5
                elif(action == Position.L_PICKUP_POS):
                    self.move_to_coords('left',pos[5]) #6
                elif(action == Position.R_ADAPT_POS):
                    # this command is for the adaptive positioning 
                    current = self._left_arm.endpoint_pose()
                    self.adaptive_coords('right',pos[6],current['position']) #7
                elif(action == Position.L_MEET_POS):
                    self.move_to_coords('left',pos[7]) #8
                elif(action == Position.L_HYB_MOV):
                    # this command is for the hybrid movment
                    self.hybrid_movement()              #9
                elif(action == Position.NEUTRAL_POS):
                    self.set_neutral()                  #0
                else:
                    print("**Invald Command**")
                
                self.set_wrench_bias()

            rate.sleep()
            

def main():
    """
    configure the directory to read the correct position file in
    along with setting uo the object 
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        "-f", "--file", required = True, 
        help = "Provide a file to read form"
    )
    # collects the name of the position text file from command line arg
    args = parser.parse_args(rospy.myargv()[1:])

    # change directory to where the position text file is
    path = "/home/baxter/ros_ws_JI/launch"
    # checks if path is vaild
    if(os.path.isdir(path)):
        os.chdir(path)

    # initalizes the ROS node
    rospy.init_node("position_movment")

    # sets up the object and takes in the arg text file
    move = Movement(args.file)

    # confuring shutdown proceeder 
    exiting = move.clean_shutdown
    rospy.on_shutdown(exiting)

    # if not shuting down then run the movement control loop
    if(exiting != True):
        move.movement()
        

    rospy.loginfo('%s::Baxter: Done.'%(rospy.get_name()))
    return 0

if __name__ == '__main__':
    main()
