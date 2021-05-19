#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Example
"""
import argparse
import struct
import sys

import rospy

import baxter_dataflow
from baxter_interface import Gripper
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from std_msgs.msg import Float64

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)





def ik_test(limb):
    rospy.init_node("rsdk_ik_service_client")

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    left.open()

    print("insert x")
    x1=input()
    print("insert y")
    y1=input()
    print("insert z")
    z1=input()
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x1,
                    y=y1,
                    z=z1
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }
    pose1 = PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=x1,
                    y=y1,
                    z=z1
                ),
                orientation=Quaternion(
                    x=0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        )

    #ikreq.pose_stamp.append(poses[limb])
    ikreq.pose_stamp.append(pose1)
    try:
        rospy.wait_for_service(ns, 5.0)
        ikreq.seed_mode = 0
        print (ikreq)
        resp = iksvc(ikreq)
        
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print(ikreq)
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        print "------------------"
        print "------------------"
        print "------------------"
        print "joints", resp.joints[0].name
        print "------------------"
        print "------------------"
        print "------------------"
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
        
        pub1 = rospy.Publisher('/robot/left_gripper_controller/joints/l_gripper_r_finger_controller/command', Float64, queue_size=1) 
        gr1=Float64()
        gr1.data= -1        
        while pub1.get_num_connections() < 1:
            it=1
        pub1.publish(gr1)
        pub2 = rospy.Publisher('/robot/left_gripper_controller/joints/l_gripper_l_finger_controller/command', Float64, queue_size=1) 
        gr2=Float64()
        gr2.data= 1        
        while pub2.get_num_connections() < 1:
            it=1
        pub2.publish(gr2)
        
        pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=1)  

        jointss=JointCommand()
        jointss.mode= 1
        jointss.names= resp.joints[0].name
        jointss.command= resp.joints[0].position
        while pub.get_num_connections() < 1:
            it=1
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

        pub.publish(jointss)

        rospy.sleep(1)
        pub1 = rospy.Publisher('/robot/left_gripper_controller/joints/l_gripper_r_finger_controller/command', Float64, queue_size=1) 
        gr1=Float64()
        gr1.data= 0       
        while pub1.get_num_connections() < 1:
            it=1
        pub1.publish(gr1)
        pub2 = rospy.Publisher('/robot/left_gripper_controller/joints/l_gripper_l_finger_controller/command', Float64, queue_size=1) 
        gr2=Float64()
        gr2.data= 0       
        while pub2.get_num_connections() < 1:
            it=1
        pub2.publish(gr2)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    

    return 0


def main():
    """RSDK Inverse Kinematics Example

    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.

    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    
    
    
    return ik_test(args.limb)

if __name__ == '__main__':
    sys.exit(main())
