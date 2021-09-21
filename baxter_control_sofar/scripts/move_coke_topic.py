#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState

"""
Move coke model
This script is used to move in the environment the model of a coke in the Baxter's frame, which will be used to visualize where the goal position of the EE of Baxter will be located.
To move the model is necessary to launch the script and then publish on the position's topic:

rosrun baxter_control_sofar move_coke_topic.py
rostopic pub /coke_can_cods geometry_msgs/Point "x: 0.5
    y: 0.4
    z: 0.8" 

"""
global pub_position

def callback(msg_point):
    
    print("moving coke to ", msg_point)

    msg = ModelState()
    msg.model_name = 'coke_can2'
    msg.pose.position.x=msg_point.x
    msg.pose.position.y=msg_point.y
    msg.pose.position.z=msg_point.z+0.9  #move the cokec in the baxter frame, slight adjustment on the z axis
    pub_position.publish(msg)
    #print(msg)
    
def listener():

    '''
        Republishes the point received as a ModelState to move the coke model
    '''

    rospy.init_node('move_coke', anonymous=True)
    global pub_position
    pub_position = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

    rospy.Subscriber("/coke_can_coords", Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()