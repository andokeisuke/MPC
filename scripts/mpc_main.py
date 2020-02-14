#!/usr/bin/env python

import agent_class
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import tf


def main():

        
	rospy.init_node('mpc')
	listener = tf.TransformListener()
	cmd = Twist()
	state = Pose2D()
	publisher = rospy.Publisher('cmd_vel',Twist, queue_size=10)
	publisher2 = rospy.Publisher('pose',Pose2D, queue_size=10)

	rate = rospy.Rate(10.0)

	controller = agent_class.NMPCController_with_CGMRES()


	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		state.x = trans[0]
		state.y = trans[1]
		e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))

		state.theta = e[2]
		publisher2.publish(state)


		#cal_input
		i = 1
		time = float(i) #* dt
		u_1s, u_2s = controller.calc_input(state.x, state.y, state.theta, time)
		


		
		
		cmd.linear.x = 1#u_1s[0]
		#cmd.linear.y = 0#u_2s[1]
		cmd.angular.z = 0#u_2s[0]
		
		publisher.publish(cmd)
		i = i + 1

		
		
'''	
    # simulation time
    dt = 0.01
    iteration_time = 15.
    iteration_num = int(iteration_time/dt)

    # controller
    controller = NMPCController_with_CGMRES()

    # for i in range(iteration_num)
    for i in range(1, iteration_num):
        time = float(i) * dt
        x_1 = plant_system.x_1
        x_2 = plant_system.x_2
        x_3 = plant_system.x_3
        # make input
        u_1s, u_2s = controller.calc_input(x_1, x_2, x_3, time)
        # update state
        plant_system.update_state(u_1s[0], u_2s[0])
    
 ''' 


if __name__ == "__main__":
    main()

