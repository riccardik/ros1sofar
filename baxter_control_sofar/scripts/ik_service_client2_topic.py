#!/usr/bin/env python



"""
Baxter Inverse Kinematics
This script can be used to send EE goals to the Baxter robot and use its internal engine for Inverse Kinematics to generate the joint's position to reach it

rosrun baxter_examples ik_service_client2_topic.py -l left
rostopic pub position_sub geometry_msgs/Point "x: 0.8
y: 0.8
z: 0.03" 

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
from std_msgs.msg import String

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)



global limb
global req_pos
limb="left"
global rs 
def endpoint_pos_callback(data):
    position = Point()
    position= data.pose.position
    #print (position)
    pub_endpoint = rospy.Publisher('left_endpoint_pos', Point, queue_size=1)
    pub_endpoint.publish(position)

    

def pose_callback(data):
    global req_pos
    req_pos = data
    print(req_pos)

    """ rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled """
    global rs
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    left.open()

    
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    
    q1 = Quaternion(
                    x=0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                )
    q2 = Quaternion(
                    x=-0.36,
                    y=0.88,
                    z=0.10,
                    w=0.259,
                )
   
    pose1 = PoseStamped(
            header=hdr,
            pose=Pose(
                position=req_pos,
                orientation=q1,
            ),
        )

    #ikreq.pose_stamp.append(poses[limb])kk
    ikreq.pose_stamp.append(pose1)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    esit_pub = rospy.Publisher('/ik_result', String, queue_size=1)

    pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=1)

    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] == resp.RESULT_INVALID):
        print("Trying from an other position")
        pubj = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=1)  
        #put joint in a decent seed position
        jointss=JointCommand()
        jointss.mode= 1
        jointss.names= ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
        jointss.command= [0.3202501782043669, 0.9507986021454259, -2.697083727757709, 1.662214963899674, 2.955581123794093, 1.6476877105908354, -0.19474867975667742]       

        pub.publish(jointss)
        rospy.sleep(1)

        ikreq.pose_stamp.append(pose1)
        try:
            rospy.wait_for_service(ns, 5.0)
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
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        msg_s = String()
        msg_s.data="SUCCESS"
        esit_pub.publish(msg_s)
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
        
          

        jointss=JointCommand()
        jointss.mode= 1
        jointss.names= resp.joints[0].name
        jointss.command= resp.joints[0].position
        while pub.get_num_connections() < 1:
            it=1
            # wait for a connection to publisher

        pub.publish(jointss)

        rospy.sleep(1)
        #open gripper
        pub1 = rospy.Publisher('/robot/left_gripper_controller/joints/l_gripper_r_finger_controller/command', Float64, queue_size=1) 
        gr1=Float64()
        gr1.data= 0       
        while pub1.get_num_connections() < 1:
            it=1
        pub1.publish(gr1)
        #close gripper
        pub2 = rospy.Publisher('/robot/left_gripper_controller/joints/l_gripper_l_finger_controller/command', Float64, queue_size=1) 
        gr2=Float64()
        gr2.data= 0       
        while pub2.get_num_connections() < 1:
            it=1
        pub2.publish(gr2)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        msg_s = String()
        msg_s.data="FAILED"
        esit_pub.publish(msg_s)

def ik_test(limb):
    rospy.init_node("rsdk_ik_service_client")
    rospy.Subscriber("position_sub", Point, pose_callback)
    rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState,endpoint_pos_callback)
    global rs
    #enable the robot
    rs = baxter_interface.RobotEnable()
    rs.enable()
    pubj = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=1)  
    #initialize joints position
    jointss=JointCommand()
    jointss.mode= 1
    jointss.names= ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
    jointss.command= [0.3202501782043669, 0.9507986021454259, -2.697083727757709, 1.662214963899674, 2.955581123794093, 1.6476877105908354, -0.19474867975667742]
    rospy.sleep(3)

    pubj.publish(jointss)
    rospy.sleep(1)
    while 1:
        rospy.sleep(1)

    

    return 0


def main():
    """RSDK Inverse Kinematics 

    
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=False,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    
    
    
    return ik_test(args.limb)

if __name__ == '__main__':
    sys.exit(main())
