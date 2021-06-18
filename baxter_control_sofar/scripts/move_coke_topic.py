#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState
global pub_position
def callback(msg_point):
    
    print("moving coke to ", msg_point)

    msg = ModelState()
    msg.model_name = 'coke_can2'
    msg.pose.position.x=msg_point.x
    msg.pose.position.y=msg_point.y
    msg.pose.position.z=msg_point.z
    pub_position.publish(msg)
    #print(msg)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('move_coke', anonymous=True)
    global pub_position
    pub_position = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    rospy.Subscriber("/coke_can_coords", Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()