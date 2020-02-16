#!/usr/bin/env python

import str_agent_class
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
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
    state_ref.x = rospy.get_param("str_sim_node/initial_x")
    state_ref.y = rospy.get_param("str_sim_node/initial_y")
    state_ref.theta = rospy.get_param("str_sim_node/initial_th")
    N=2
    
        

    publisher = rospy.Publisher('cmd_vel',Twist, queue_size=10)
    pub3 = rospy.Publisher('path',Path, queue_size=10)
    sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
    
    rate = rospy.Rate(100.0)
    controller = str_agent_class.NMPCController_with_CGMRES()
    i = 0
    
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
       
        time = float(i*0.01) #* dt
        if(cmd_pose!=controller.pre_cmd_pose):
            i = 0

        u_1s, u_2s, u_3s,x_1s, x_2s= controller.calc_input(state.x, state.y, state.theta,cmd_pose, time)
        controller.pre_cmd_pose = cmd_pose

        path = Path()
        path.header.frame_id = "world" 
        path.header.stamp = rospy.Time.now()
        path.header.seq = 0;

        for i in range(controller.N):
            p = PoseStamped()
            p.header.frame_id = "world" 
            p.header.stamp = rospy.Time.now()
            p.header.seq = 0;
            p.pose.position.x = x_1s[i]
            p.pose.position.y = x_2s[i]
            path.poses.append(p)

        if(u_1s[0]>N):
            u_1s[0]=N
        if(u_1s[0]<-N):
            u_1s[0]=-N
        if(u_2s[0]>N):
            u_2s[0]=N
        if(u_2s[0]<-N):
            u_2s[0]=-N
        cmd.linear.x = u_1s[0]
        cmd.linear.y = u_2s[0]
        cmd.angular.z = u_3s[0]
        
        publisher.publish(cmd)
        pub3.publish(path)
        i = i + 1
        #r.sleep()



if __name__ == "__main__":
    main()

