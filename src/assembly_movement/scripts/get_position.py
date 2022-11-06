#!/usr/bin/env python

import argparse
import rospy
import numpy as np

import baxter_interface

def Monitor():
    left_arm = baxter_interface.limb.Limb('left')
    right_arm = baxter_interface.limb.Limb('right')
    current_left_arm = left_arm.endpoint_pose()
    current_right_arm = right_arm.endpoint_pose()
    rospy.loginfo("left_arm pose: \n %s", current_left_arm)
    rospy.loginfo("right_arm pose: \n %s", current_right_arm)

def main():
    """
    Commands get end-effectr position 
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("joint_monitor")

    Monitor()
    rospy.spin()

    print("Done")

if __name__ == '__main__':
    main()