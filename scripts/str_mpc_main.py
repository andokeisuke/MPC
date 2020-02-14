#!/usr/bin/env python

import str_agent_class
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf

state_ref = Pose2D()
publisher2 = rospy.Publisher('pose',Pose2D, queue_size=10)

def quaternion_to_yaw(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return e[2]

    
def callback(goals):

    state_ref.x = goals.pose.position.x
    state_ref.y = goals.pose.position.y
    state_ref.theta = quaternion_to_yaw(goals.pose.orientation)
    publisher2.publish(state_ref)




def main():
    
    rospy.init_node('mpc')
    listener = tf.TransformListener()
    cmd = Twist()
    state = Pose2D()

    publisher = rospy.Publisher('cmd_vel',Twist, queue_size=10)
    sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
    
    rate = rospy.Rate(10.0)
    controller = str_agent_class.NMPCController_with_CGMRES()
    
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            
            continue
        state.x = trans[0]
        state.y = trans[1]
        e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
        state.theta = e[2]
        
        
       
        

        cmd_pose = [state_ref.x, state_ref.y, state_ref.theta]
        i = 1
        time = float(i) #* dt
        
        u_1s, u_2s, u_3s= controller.calc_input(state.x, state.y, state.theta,cmd_pose, time)
        
        cmd.linear.x = u_1s[0]
        cmd.linear.y = u_2s[0]
        cmd.angular.z = u_3s[0]
        
        publisher.publish(cmd)
        i = i + 1


if __name__ == "__main__":
    main()

