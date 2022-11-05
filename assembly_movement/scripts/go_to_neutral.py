#!/usr/bin/env python

import argparse
import rospy
import numpy as np

from std_msgs.msg import (UInt16)

import baxter_interface

from baxter_interface import CHECK_VERSION


class Move(object):
    
    def __init__(self):
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_arm = baxter_interface.limb.Limb("left")


        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def _reset_control_modes(self):
        rate = rospy.Rate(100)
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
        print("Moving to neutral pose...")
        self._right_arm.move_to_neutral()
        self._left_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

def main():
    """Moves Baxter to Neutral position.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("move_to_neutral")

    neut = Move()
    rospy.on_shutdown(neut.clean_shutdown)
    neut.set_neutral()

    print("Done.")

if __name__ == '__main__':
    main()
