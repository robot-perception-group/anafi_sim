#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf
import numpy as np
from std_msgs.msg import Float64, Bool, String
from anafi_control.msg import Waypoint
from olympe_bridge.msg import MoveToCommand
import pymap3d
from anafi_control.msg import Waypoint

from numpy.polynomial import Chebyshev as Ch
import matplotlib.pyplot as plt


#Parameters
node_name ='trajectory_generator_node' 
drone_name = rospy.get_param(rospy.get_name()+'/drone_name','anafi')   
publish_hz = float(rospy.get_param(rospy.get_name()+'/publish_hz','10'))
prefix = "/trajectory_generator"

#Define topics
waypoints_topic = ('/'+drone_name+'/position_control/waypoint',Waypoint)
static_waypoint_topic = (prefix+'/static_waypoint',Waypoint)
trajectory_begin_timer_topic = (prefix+'/begin',Bool)
moveto_topic = ('/'+drone_name+'/drone/moveto_remove_this_suffix',MoveToCommand)


class GenerateTrajectory():
    def __init__(self):
        self.t = np.arange(-1,1,0.004)

        self.a_x = self.a_long(self.t)
        self.a_y = self.a_lat(self.t)
        self.a_z = self.a_vert(self.t)

        self.v_x = np.clip(2*self.simpson_integration(0.5,self.a_x,0.1)-3,-10,10)
        self.v_y = np.clip(0.5*self.simpson_integration(2,self.a_y,0.1),-10,10)
        self.v_z = np.clip(self.simpson_integration(-2,self.a_z,0.1),-2,2)

        self.p_x = self.simpson_integration(0,self.v_x,0.1)
        self.p_y = self.simpson_integration(0,self.v_y,0.1)
        self.p_z = self.simpson_integration(20,self.v_z,0.1)

        
    def a_long(self,x):
        p = 9*x**4 - 5*x + 5
        return 2*(np.tanh(p)+np.cos(12*x**2-2))/2  -1

    def a_lat(self,x):
        p = 10*x**4 -3*x**3- 2*x + -2

        return np.clip(2*((np.tanh(p)  +0.8*np.sin(8*x**5))/2),-5,5)+0


    def a_vert(self,x):
        p = 1.5*x**6 - 2*x + 5

        return 2*((np.tanh(p/1)  +0.8*np.sin(3*x**3)/np.cos(x))/2)

    def simpson_integration(self,y0, y, h):
        y = np.asarray(y)
        n = y.size
        S = np.zeros(n, dtype=float)
        if n == 0:
            return S
        S[0] = y0
        if n == 1:
            return S

        # First step: trapezoid
        S[1] = S[0] + 0.5*h*(y[0] + y[1])

        # Advance in blocks of two with Simpson; fill odd indices by trapezoid from the last even
        for k in range(2, n, 2):
            S[k] = S[k-2] + (h/3.0)*(y[k-2] + 4.0*y[k-1] + y[k])
            if k + 1 < n:
                S[k+1] = S[k] + 0.5*h*(y[k] + y[k+1])

        return S


class TrajectoryGenerator:
    """Class that creates a trajectory and publishes it using geometry_msgs/Pose messages"""

    def __init__(self):
        """Class contains data required in order to update the current position of the moving platform."""        
        #Get parameters

        self.trajectory_start_position = rospy.get_param(rospy.get_name()+"/trajectory_start_position",{'x':0,'y':0,'z':10}) #[m]
        self.trajectory_start_orientation = rospy.get_param(rospy.get_name()+"/trajectory_start_orientation",{'phi':0,'theta':0,'psi':0}) #{rad}




        #Convert dict values to float
        for k, v in self.trajectory_start_position.items():
            self.trajectory_start_position[k] = float(v)

        for k, v in self.trajectory_start_orientation.items():
            self.trajectory_start_orientation[k] = float(v)


        self.w = 0

        #orientation in Euler angles
        self.phi  = self.trajectory_start_orientation['phi']
        self.theta = self.trajectory_start_orientation['theta']
        self.psi = self.trajectory_start_orientation['psi']

        #Time parameters
        self.t = 0
        self.t_max = 50 #sec
        self.delta_t = 1 / publish_hz #sec  


        #Publishers
        self.waypoints_publisher = rospy.Publisher(waypoints_topic[0],waypoints_topic[1],queue_size = 1)
        self.timer_publisher = rospy.Publisher(trajectory_begin_timer_topic[0],trajectory_begin_timer_topic[1],queue_size=1)
        self.moveto_publisher = rospy.Publisher(moveto_topic[0],moveto_topic[1],queue_size=1)

        #Subscribers
        self.begin_sent = rospy.Subscriber(trajectory_begin_timer_topic[0],trajectory_begin_timer_topic[1],self.read_begin)
        

        #variables
        self.tg = GenerateTrajectory()
        self.v_x_wp = 0
        self.v_y_wp = 0
        self.v_z_wp = 0

        self.p_x_start = 0
        self.p_y_start = 0
        self.p_z_start = 7

        self.begin_sent = False

        return
    
    def read_begin(self,msg):
        self.begin_sent = True
    

        
    def publish_trajectory(self,idx):
        msg = Waypoint()
        msg.x = self.tg.p_x[idx]
        msg.y = self.tg.p_y[idx]
        msg.z = self.tg.p_z[idx]
        msg.v_x = self.tg.v_x[idx]
        msg.v_y = self.tg.v_y[idx]
        msg.v_z = self.tg.v_z[idx]
        self.waypoints_publisher.publish(msg)
        print(msg)
        print("idx =",idx)
        return


if __name__ == '__main__':
    rospy.init_node(node_name, anonymous=True)
    trajectory_generator = TrajectoryGenerator()
    
    rate = rospy.Rate(publish_hz)
    idx = 1
    idx_max = len(trajectory_generator.tg.p_x)-1
    count_upwards = True
    while not rospy.is_shutdown():
        trajectory_generator.publish_trajectory(idx)
        if not trajectory_generator.begin_sent:
            print("\033[93m I am here\033[0m")
            trajectory_generator.timer_publisher.publish(Bool(True))


        if idx >= idx_max:
            trajectory_generator.timer_publisher.publish(Bool(True))
            rospy.signal_shutdown("End reached")




        idx +=1 
    

        



        rate.sleep()


    
