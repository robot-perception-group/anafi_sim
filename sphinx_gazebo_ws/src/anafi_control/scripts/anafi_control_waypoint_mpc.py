#!/home/raven/venvs/sphinx_with_gazebo/bin/python
import osqp
import numpy as np
from scipy import sparse

import rospy
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Vector3Stamped, Vector3
import tf2_ros
import tf2_geometry_msgs 
from sphinx_with_gazebo.msg import Sphinx
from anafi_control.msg import Waypoint
from olympe_bridge.msg import PilotingCommand
from anafi_control.msg import State
from tf.transformations import euler_from_quaternion




#Parameters
node_name = "anafi_control_waypoint_mpc_node"
publish_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/publish_hz',"50"))
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name',"anafi")


#General Topics
waypoint_topic = ('/'+drone_name+'/position_control/waypoint',Waypoint)
uav_state_topic = ('/'+drone_name+'/position_control/state_enu',State)

yaw_pid_state_topic =          ('/'+drone_name+'/yaw_pid/state',Float64)
yaw_pid_setpoint_topic =       ('/'+drone_name+'/yaw_pid/setpoint',Float64)
yaw_pid_control_effort_topic = ('/'+drone_name+'/yaw_pid/control_effort',Float64)

rpyt_topic = ('/'+drone_name+'/drone/rpyt',PilotingCommand)
position_error_topic = ('/'+drone_name+'/position_error_debug',Vector3Stamped)
ext_force_debug_topic = ('/'+drone_name+'/ext_force_debug',Vector3Stamped)

x_state_debug_topic = ('/'+drone_name+'/x_state_debug',Float64MultiArray)
x_desired_debug_topic = ('/'+drone_name+'/x_desired_debug',Float64MultiArray)
eint_x_debug_topic = ('/'+drone_name+'/eint_x_debug',Float64MultiArray)
eint_y_debug_topic = ('/'+drone_name+'/eint_y_debug',Float64MultiArray)
eint_z_debug_topic = ('/'+drone_name+'/eint_z_debug',Float64MultiArray) 
eint_vx_debug_topic = ('/'+drone_name+'/eint_vx_debug',Float64MultiArray)
eint_vy_debug_topic = ('/'+drone_name+'/eint_vy_debug',Float64MultiArray)
eint_vz_debug_topic = ('/'+drone_name+'/eint_vz_debug',Float64MultiArray)
trajectory_x_debug_topic = ('/'+drone_name+'/trajectory_x_debug',Float64MultiArray)
trajectory_y_debug_topic = ('/'+drone_name+'/trajectory_y_debug',Float64MultiArray)
trajectory_z_debug_topic = ('/'+drone_name+'/trajectory_z_debug',Float64MultiArray)
trajectory_vx_debug_topic = ('/'+drone_name+'/trajectory_vx_debug',Float64MultiArray)
trajectory_vy_debug_topic = ('/'+drone_name+'/trajectory_vy_debug',Float64MultiArray)
trajectory_vz_debug_topic = ('/'+drone_name+'/trajectory_vz_debug',Float64MultiArray)


class MPCOSQPWaypoint():
    def __init__(self):

        self.set_mpc_parameters()
        self.set_script_parameters()
        self.set_up_subscribers_and_publishers()
        self.set_up_mpc_problem()
        self.set_up_mpc_solution_framework()
        print("Done setting up mpc problem.")
        return

    def set_up_subscribers_and_publishers(self):
        #Debug publishers
        #Subscribers
        self.uav_state_subscriber = rospy.Subscriber(uav_state_topic[0],uav_state_topic[1],self.read_uav_state)
        self.waypoint_subscriber = rospy.Subscriber(waypoint_topic[0],waypoint_topic[1],self.read_waypoint)
        self.yaw_pid_control_effort_susbcriber = rospy.Subscriber(yaw_pid_control_effort_topic[0],yaw_pid_control_effort_topic[1],self.read_yaw_pid_control_effort)
        
        #publishers
        self.rpyt_publisher = rospy.Publisher(rpyt_topic[0],rpyt_topic[1],queue_size=0)
        self.position_error_debug_publisher = rospy.Publisher(position_error_topic[0],position_error_topic[1],queue_size=0)
        self.ext_force_publisher = rospy.Publisher(ext_force_debug_topic[0],ext_force_debug_topic[1],queue_size=0)
        self.yaw_pid_state_publisher = rospy.Publisher(yaw_pid_state_topic[0],yaw_pid_state_topic[1],queue_size=0)
        self.yaw_pid_setpoint_publisher = rospy.Publisher(yaw_pid_setpoint_topic[0],yaw_pid_setpoint_topic[1],queue_size=0)
        self.x_state_debug_publisher = rospy.Publisher(x_state_debug_topic[0],x_state_debug_topic[1],queue_size=0)
        self.x_desired_debug_publisher = rospy.Publisher(x_desired_debug_topic[0],x_desired_debug_topic[1],queue_size=0)
        self.eint_x_debug_publisher = rospy.Publisher(eint_x_debug_topic[0],eint_x_debug_topic[1],queue_size=0)
        self.eint_y_debug_publisher = rospy.Publisher(eint_y_debug_topic[0],eint_y_debug_topic[1],queue_size=0)
        self.eint_z_debug_publisher = rospy.Publisher(eint_z_debug_topic[0],eint_z_debug_topic[1],queue_size=0)
        self.eint_vx_debug_publisher = rospy.Publisher(eint_vx_debug_topic[0],eint_vx_debug_topic[1],queue_size=0)
        self.eint_vy_debug_publisher = rospy.Publisher(eint_vy_debug_topic[0],eint_vy_debug_topic[1],queue_size=0)
        self.eint_vz_debug_publisher = rospy.Publisher(eint_vz_debug_topic[0],eint_vz_debug_topic[1],queue_size=0)

        self.trajectory_x_debug_publisher = rospy.Publisher(trajectory_x_debug_topic[0],trajectory_x_debug_topic[1],queue_size=0)
        self.trajectory_y_debug_publisher = rospy.Publisher(trajectory_y_debug_topic[0],trajectory_y_debug_topic[1],queue_size=0)
        self.trajectory_z_debug_publisher = rospy.Publisher(trajectory_z_debug_topic[0],trajectory_z_debug_topic[1],queue_size=0)
        self.trajectory_vx_debug_publisher = rospy.Publisher(trajectory_vx_debug_topic[0],trajectory_vx_debug_topic[1],queue_size=0)
        self.trajectory_vy_debug_publisher = rospy.Publisher(trajectory_vy_debug_topic[0],trajectory_vy_debug_topic[1],queue_size=0)        
        self.trajectory_vz_debug_publisher = rospy.Publisher(trajectory_vz_debug_topic[0],trajectory_vz_debug_topic[1],queue_size=0)
        return

    def set_mpc_parameters(self):
        # Params
        #mpc params
        self.dt = 0.1
        self.N = 30
        self.amin = -np.array([5,5,5]) #x,y - acceleration limits, should correspond to limit attitude angle, used for MPC input
        self.amax =  np.array([5,5,5]) #x,y - acceleration limits, should correspond to limit attitude angle, used for MPC input



        self.vmin = -np.array([10,10,2]) #x,y,z - velocity limits, z-component is used for MPC input
        self.vmax =  np.array([10,10,2]) #x,y,z - velocity limits, z-component is used for MPC input

        #Don't use hard constraints on the integral terms
        self.eint_min = np.array([-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
        self.eint_max = np.array([ np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

                                #      x_ref        x      y_ref      y         z_ref           z         vx_ref  vx     vy_ref      vy    vz_ref      vz    x_int   y_int  z_int    vx_int  vy_int    vz_int   
        # self.weights_Q  =   np.array([ 0.0 ,   0.0,   0.0,    0.0,       0.0,        0.0,       0e1,   0e1,    0e1,       0e1,   0e1,       0e1,   0.4,    0.4,   6,       0,       0,    0e0])
        # self.weights_QN =   np.array([ 0.0 ,   0.0,   0.0,    0.0,       0.0,        0.0,       0e1,   0e1,    0e1,       0e1,   0e1,       0e1,   3e1,    3e1,   3e1,       0e1,    0e1,   0e0])

                               #      x_ref    x      y_ref   y         z_ref        z         vx_ref  vx     vy_ref      vy    vz_ref      vz    x_int   y_int  z_int    vx_int  vy_int    vz_int          
        self.weights_Q  =   np.array([ 0.0 ,   0.0,   0.0,    0.0,       0.0,        0.0,       0e1,   0e1,    0e1,       0e1,   0e1,       0e1,   0.001,     0.001,   0.001,       0,      0,     0e0])
        self.weights_QN =   np.array([ 0.0 ,   0.0,   0.0,    0.0,       0.0,        0.0,       0e1,   0e1,    0e1,       0e1,   0e1,       0e1,   5e0,    5e0,   5e0,       0e1,    0e1,   0e0])
        
        
        self.weight_R = 0.2

        #Solver params
        self.solver_settings = {}
        self.solver_settings["max_iter"] = 4000
        self.solver_settings["verbose"] = False

        #Run parames
        self.execution_freq = 10   

        #Init variables
        self.xend = np.zeros(18)
        self.xbegin = np.ones(18)
        return

    def set_script_parameters(self):
        #Waypoint
        self.waypoint = np.zeros(7)
        self.x_desired = np.zeros(18)
        #Current state
        self.x_state = np.zeros(18) # in stability axes, x_state = [x_ref,x,y_ref,y,vx_ref,vx,vy_ref,vy_x_eint.y_eint,vx_eint,vy_eint]

        #Transformation
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #Pid
        self.yaw_control_effort = 0
        self.yaw_control_setpoint = 0
        self.yaw_control_state = 0

        self.pos_error_interval = 0.1 #s
        self.pos_error_list = int(self.pos_error_interval*100)*[np.array([0,0,0])]
        return

    def set_up_mpc_problem(self):
        # Discrete time model of a quadcopter
        
        #State x = [x_ref,x,y_ref,y,z_ref,z,vx_ref,vx,vy_ref,vy,vz_ref,vz]
        #Augmented state x_aug (used for MPC) x_aug = [x,x]
        #Augmented state x_aug = [x,z], where z = [x_int_error,y_int_error,z_int_error,vx_int_error,vy_int_error,vz_int_error]]


        self.Ad = sparse.csc_matrix(np.block([
            #Update of states x
            [np.eye(6), self.dt*np.eye(6),np.zeros((6,6))], #update of positional states
            [np.zeros((6,6)),np.eye(6),np.zeros((6,6))],    #update of velocity states
            #Update of integral error states, z
            [np.array([
                [ 1,-1, 0, 0, 0, 0, 0, 0, 0,0,0,0,  1, 0, 0, 0, 0, 0 ], # relative position in x-direction
                [ 0, 0, 1,-1, 0, 0, 0, 0, 0,0,0,0,   0, 1, 0, 0, 0, 0], # relative position in y-direction
                [ 0, 0, 0, 0, 1,-1, 0, 0, 0, 0, 0, 0,  0, 0, 1, 0, 0, 0], # relative position in z-direction
                [ 0, 0, 0, 0, 0, 0, 1,-1, 0, 0 ,0, 0, 0, 0, 0, 1, 0, 0], #relative velocity in x-direction
                [ 0, 0, 0, 0, 0, 0, 0, 0, 1,-1 ,0, 0, 0, 0, 0, 0, 1, 0], #relative velocity in y-direction
                [ 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0 ,1,-1, 0, 0, 0, 0, 0, 1], #relative velocity in z-direction
            ])]
        ]))
        self.Bd = sparse.csc_matrix(np.block([
            [np.array([
                [0,0,0],            #update for x_ref
                [self.dt**2/2,0,0], #update for x. Input is an acceleration
                [0,0,0],            #update for y_ref
                [0,self.dt**2/2,0], #update for y. Input is an acceleration
                [0,0,0],            #update for z_ref
                [0,0,self.dt**2/2],      #update for z. Input is a velocity
                [0,0,0],            #update for vx_ref 
                [self.dt,0,0],      #update for vx. Input is an acceleration
                [0,0,0],            #update for vy_ref 
                [0,self.dt,0],      #update for vy. Input is an acceleration
                [0,0,0],            #update for vz_ref 
                [0,0,  self.dt],            #update for vz. Input is a velocity
            ])],
            [np.zeros((6,3))] #update for integration states


            ]))



        [self.nx, self.nu] = self.Bd.shape
        print("===========>nx,nu",self.nx,self.nu)


        #Define reference state trajectory that needs to be tracked
        self.xr = np.ones((18,self.N+1))
        self.xr[0,:] = self.xend[0] * self.xr[0,:]
        self.xr[1,:] = self.xend[1] * self.xr[1,:]
        self.xr[2,:] = self.xend[2] * self.xr[2,:]
        self.xr[3,:] = self.xend[3] * self.xr[3,:]
        self.xr[4,:] = self.xend[4] * self.xr[4,:]
        self.xr[5,:] = self.xend[5] * self.xr[5,:]
        self.xr[6,:] = self.xend[6] * self.xr[6,:]
        self.xr[7,:] = self.xend[7] * self.xr[7,:]
        self.xr[8,:] = self.xend[8] * self.xr[8,:]
        self.xr[9,:] = self.xend[9] * self.xr[9,:]
        self.xr[10,:] = self.xend[10] * self.xr[10,:]
        self.xr[11,:] = self.xend[11] * self.xr[11,:]
        self.xr[12,:] = 0            * self.xr[12,:]   #x int state should become 0
        self.xr[13,:] = 0            * self.xr[13,:]   #y int state should become 0
        self.xr[14,:] = 0            * self.xr[14,:]   #z int state should become 0
        self.xr[15,:] = 0            * self.xr[15,:]  #vx int state should become 0
        self.xr[16,:] = 0            * self.xr[16,:]  #vy int state should become 0
        self.xr[17,:] = 0            * self.xr[17,:]  #vz int state should become 0


        #External forces
        self.x0 = self.xbegin
        self.ext_force = np.zeros((self.N,2))
        self.ext_force = self.calc_ext_force()



        # Constraints
        self.umin = self.amin
        self.umax = self.amax
        #Combine umins with external force
        umins = np.tile(self.umin,(self.N,1))
        umaxs = np.tile(self.umax,(self.N,1))
        #                       x_ref     x    y_ref     y     z_ref      z    vx_ref   vx          vy_ref   vy          vz_ref     vz             x_eint          y_eint          z_eint       vx_int          vy_int              vz_int
        self.xmin = np.array([-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,self.vmin[0],-np.inf,self.vmin[1],-np.inf,self.vmin[2],self.eint_min[0],self.eint_min[1],self.eint_min[2],self.eint_min[3],self.eint_min[4],self.eint_min[5]])
        self.xmax = np.array([ np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf,self.vmax[0], np.inf,self.vmax[1], np.inf,self.vmax[2],self.eint_max[0],self.eint_max[1],self.eint_max[2],self.eint_max[3],self.eint_max[4],self.eint_max[5]])


        # Objective function
        self.Q = sparse.diags(self.weights_Q)
        self.QN = sparse.diags(self.weights_QN)
        self.R = self.weight_R*sparse.csc_matrix(np.eye(3))


        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        self.P = sparse.block_diag([sparse.kron(sparse.eye(self.N), self.Q), self.QN,
                            sparse.kron(sparse.eye(self.N), self.R)], format='csc')
        # - linear objective
        self.q = np.hstack([-self.Q.dot(self.xr[:,:self.N]).flatten(), -self.QN.dot(self.xr[:,self.N]),
                    np.zeros(self.N*self.nu)])
        
        print("np.shape(self.q)",np.shape(self.q))
        # - linear dynamics
        self.Ax = sparse.kron(sparse.eye(self.N+1),-sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N+1, k=-1), self.Ad)
        self.Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), self.Bd)
        self.Aeq = sparse.hstack([self.Ax, self.Bu])
        self.leq = np.hstack([-self.x0, np.zeros(self.N*self.nx)])
        self.ueq = self.leq
        # - input and state constraints
        self.Aineq = sparse.eye((self.N+1)*self.nx + self.N*self.nu)
        # - OSQP constraints
        self.A = sparse.vstack([self.Aeq, self.Aineq], format='csc')
        self.lineq = np.hstack([np.kron(np.ones(self.N+1), self.xmin), umins.flatten()])
        self.uineq = np.hstack([np.kron(np.ones(self.N+1), self.xmax), umaxs.flatten()])
        self.l = np.hstack([self.leq, self.lineq])
        self.u = np.hstack([self.ueq, self.uineq])

        print("np.shape(self.nx)",self.nx)
        print("np.shape(self.Ad)",np.shape(self.Ad))
        print("np.shape(self.Bd)",np.shape(self.Bd))
        print("np.shape(self.xmin)",np.shape(self.xmin))
        print("np.shape(self.xmax)",np.shape(self.xmax))
        print("np.shape(self.Q)",np.shape(self.Q))
        print("np.shape(self.QN)",np.shape(self.QN))
        print("np.shape(self.lineq)",np.shape(self.lineq))
        print("np.shape(self.uineq)",np.shape(self.uineq))
        print("np.shape(self.l)",np.shape(self.l))
        print("np.shape(self.u)",np.shape(self.u))
        print("np.shape(self.P)",np.shape(self.P))

        print("np.shape(self.Ax)",np.shape(self.Ax))
        print("np.shape(self.Bu)",np.shape(self.Bu))

        return
    
    def set_up_mpc_solution_framework(self):


        # Create an OSQP object
        self.prob = osqp.OSQP()

        # Setup workspace
        self.prob.setup(self.P, self.q, self.A, self.l, self.u, warm_start=True)
        return

    def solve_mpc_problem(self):
        #Update problem
        self.prob.update_settings(**self.solver_settings)
        #Solve problem
        res = self.prob.solve()

        solve_successful = True
        # Check solver status
        if res.info.status not in ['solved','solved inaccurate']:
            #raise ValueError('OSQP did not solve the problem!')
            print("OSQP did not solve the problem!")
            solve_successful = False
            # print("res.x =",res.x)
        else:
            # print("\033[0m Found solution.")
            pass

        if np.any(res.x == None):
            print(rospy.Time.now().to_sec(),": Nan found")
            print("\033[93m Could not find solution. Aborting...")
            # exit()
        else:

            #Get found trajectory of states
            self.x_trajectory = np.zeros((self.N+1,len(self.x0)))
            self.u_trajectory = np.zeros((self.N,3))
            #Insert first state
            self.x_trajectory[0,:] = self.x0
            #Calculate state trajectory
            for j in range(0,self.N):
                #If j is the last element, general indexing scheme checked for correctness in demo example.
                if j==self.N-1:
                    ctrl = res.x[-self.nu:]
                else:
                    ctrl = res.x[-(self.N-j)*self.nu:-(self.N-1-j)*self.nu]

                self.x_trajectory[j+1,:] = self.Ad.dot(self.x_trajectory[j]) + self.Bd.dot(ctrl)
                self.u_trajectory[j,:] = ctrl


            msg_eint_x = Float64MultiArray()
            msg_eint_y = Float64MultiArray()
            msg_eint_z =  Float64MultiArray()
            msg_eint_vx = Float64MultiArray()
            msg_eint_vy = Float64MultiArray()
            msg_eint_vz = Float64MultiArray()
            msg_trajectory_x = Float64MultiArray()
            msg_trajectory_y = Float64MultiArray()
            msg_trajectory_z = Float64MultiArray()
            msg_trajectory_vx = Float64MultiArray()
            msg_trajectory_vy = Float64MultiArray()
            msg_trajectory_vz = Float64MultiArray()
            msg_eint_x.data = self.x_trajectory[:,12]
            msg_eint_y.data = self.x_trajectory[:,13]
            msg_eint_z.data = self.x_trajectory[:,14]
            msg_eint_vx.data = self.x_trajectory[:,15]
            msg_eint_vy.data = self.x_trajectory[:,16]
            msg_eint_vz.data = self.x_trajectory[:,17]
            msg_trajectory_x.data = self.x_trajectory[:,1]
            msg_trajectory_y.data = self.x_trajectory[:,3]
            msg_trajectory_z.data = self.x_trajectory[:,5]
            msg_trajectory_vx.data = self.x_trajectory[:,7]
            msg_trajectory_vy.data = self.x_trajectory[:,9]
            msg_trajectory_vz.data = self.x_trajectory[:,11]
            self.eint_x_debug_publisher.publish(msg_eint_x)
            self.eint_y_debug_publisher.publish(msg_eint_y)
            self.eint_z_debug_publisher.publish(msg_eint_z)
            self.eint_vx_debug_publisher.publish(msg_eint_vx)
            self.eint_vy_debug_publisher.publish(msg_eint_vy)
            self.eint_vz_debug_publisher.publish(msg_eint_vz)
            self.trajectory_x_debug_publisher.publish(msg_trajectory_x)
            self.trajectory_y_debug_publisher.publish(msg_trajectory_y)
            self.trajectory_z_debug_publisher.publish(msg_trajectory_z)
            self.trajectory_vx_debug_publisher.publish(msg_trajectory_vx)
            self.trajectory_vy_debug_publisher.publish(msg_trajectory_vy)
            self.trajectory_vz_debug_publisher.publish(msg_trajectory_vz)
        return solve_successful
    
    def update_x0(self):
        #To avoid invalid initialization values when copter gets to fast
        x0 = self.x_state
        x0[7]  = np.clip(x0[7],self.vmin[0],self.vmax[0])
        x0[9]  = np.clip(x0[9],self.vmin[1],self.vmax[1])
        x0[11] = np.clip(x0[11],self.vmin[2],self.vmax[2])
        #Initialize the integral error with the current error
        x0[12] = np.clip(self.x0[12],self.eint_min[0],self.eint_max[0])
        x0[13] = np.clip(self.x0[13],self.eint_min[1],self.eint_max[1])
        x0[14] = np.clip(self.x0[14],self.eint_min[2],self.eint_max[2])
        x0[15] = np.clip(self.x0[15],self.eint_min[3],self.eint_max[3])
        x0[16] = np.clip(self.x0[16],self.eint_min[4],self.eint_max[4])
        x0[17] = np.clip(self.x0[17],self.eint_min[5],self.eint_max[5])
        self.x0 = x0
        return

    def update_mpc_problem(self):
        # # Apply first control input to the plant
        self.update_x0()



        QN_update = self.QN
        Q_update = self.Q
        
        #Update MPC problem
        P_update = sparse.block_diag([sparse.kron(sparse.eye(self.N), Q_update), QN_update,
                            sparse.kron(sparse.eye(self.N), self.R)], format='csc')
        
        xr = np.ones((18,self.N+1))

        #Tracked position is updated with constant velocity model over time horizon
        xr[0,:] = self.x_desired[0] + np.arange(self.N+1)*self.dt*self.x_desired[6]
        xr[1,:] = self.x_desired[0] + np.arange(self.N+1)*self.dt*self.x_desired[6]
        xr[2,:] = self.x_desired[2] + np.arange(self.N+1)*self.dt*self.x_desired[8]
        xr[3,:] = self.x_desired[2] + np.arange(self.N+1)*self.dt*self.x_desired[8]
        xr[4,:] = self.x_desired[4] + np.arange(self.N+1)*self.dt*self.x_desired[10]
        xr[5,:] = self.x_desired[4] + np.arange(self.N+1)*self.dt*self.x_desired[10]

        xr[6,:] =  self.x_desired[6] #vx_ref
        xr[7,:] =  self.x_desired[7]
        xr[8,:] =  self.x_desired[8] #vy_ref
        xr[9,:] =  self.x_desired[9]
        xr[10,:] =  self.x_desired[10] #vz_ref
        xr[11,:] =  self.x_desired[11]
        xr[12,:] =  self.x_desired[12] #integral error state x  should be set to 0
        xr[13,:] =  self.x_desired[13] #integral error state y  should be set to 0
        xr[14,:] =  self.x_desired[14] #integral error state z  should be set to 0
        xr[15,:] =  self.x_desired[15] #integral error state vx should be set to 0
        xr[16,:] =  self.x_desired[16] #integral error state vy should be set to 0
        xr[17,:] =  self.x_desired[17] #integral error state vz should be set to 0        


       
        # print("|||||||xr[-1] =",xr[-1])
        
        q_update = np.hstack([-Q_update.dot(xr[:,:self.N]).flatten(), -QN_update.dot(xr[:,self.N]),
                    np.zeros(self.N*self.nu)])



        #Update constraints
        umins = np.tile(self.umin,(self.N,1))
        umaxs = np.tile(self.umax,(self.N,1))  
        lineq = np.hstack([np.kron(np.ones(self.N+1), self.xmin), umins.flatten()])
        uineq = np.hstack([np.kron(np.ones(self.N+1), self.xmax), umaxs.flatten()])
        l_update = np.hstack([self.leq, lineq])
        u_update = np.hstack([self.ueq, uineq])
        u_update[:self.nx] = -self.x0
        l_update[:self.nx] = np.minimum(u_update[:self.nx],-self.x0)
        self.prob.update(Px=sparse.triu(P_update).data,l=l_update, u=u_update,q=q_update)
        return

    def read_uav_state(self,msg):
        self.x_state[1]  =  msg.pose.pose.position.x
        self.x_state[3]  =  msg.pose.pose.position.y
        self.x_state[5]  =  msg.pose.pose.position.z
        self.x_state[7]  =  msg.twist.twist.linear.vector.x
        self.x_state[9]  =  msg.twist.twist.linear.vector.y
        self.x_state[11] =  msg.twist.twist.linear.vector.z
        q = np.array([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        roll,pitch,yaw = euler_from_quaternion([q[0],q[1],q[2],q[3]])
        self.yaw_control_state = yaw

        msg_pos_error = Vector3Stamped()
        msg_pos_error.header.stamp = rospy.Time.now()
        msg_pos_error.header.frame_id = "world"
        msg_pos_error.vector = Vector3(self.x_state[0]-self.x_state[1],self.x_state[2]-self.x_state[3],self.x_state[3]-self.x_state[4])
        # self.position_error_debug_publisher.publish(msg_pos_error)
        
        msg_x_state = Float64MultiArray()
        msg_x_state.data = self.x_state
        self.x_state_debug_publisher.publish(msg_x_state)
        msg_x_desired = Float64MultiArray()
        msg_x_desired.data = self.x_desired
        # self.x_desired_debug_publisher.publish(msg_x_desired)

        
        return

    
    def read_waypoint(self,msg):
        self.x_state[0] = msg.x    #update the reference state which is to be tracked  
        self.x_state[2] = msg.y    #update the reference state which is to be tracked  
        self.x_state[4] = msg.z    #update the reference state which is to be tracked  
        self.x_state[6] = msg.v_x   #update the reference state which is to be tracked 
        self.x_state[8] = msg.v_y   #update the reference state which is to be tracked 
        self.x_state[10] = msg.v_z  #update the reference state which is to be tracked
        self.x_state[12] = 0
        self.x_state[13] = 0
        self.x_state[14] = 0
        self.x_state[15] = 0
        self.x_state[16] = 0
        self.x_state[17] = 0

        self.waypoint[0] = msg.x
        self.waypoint[1] = msg.y
        self.waypoint[2] = msg.z
        self.waypoint[3] = msg.v_x
        self.waypoint[4] = msg.v_y
        self.waypoint[5] = msg.v_z
        self.waypoint[6] = msg.yaw

        self.x_desired = np.zeros(18)
        self.x_desired[0] = self.waypoint[0]
        self.x_desired[1] = self.waypoint[0]
        self.x_desired[2] = self.waypoint[1]
        self.x_desired[3] = self.waypoint[1]
        self.x_desired[4] = self.waypoint[2]
        self.x_desired[5] = self.waypoint[2]
        self.x_desired[6] = self.waypoint[3]
        self.x_desired[7] = self.waypoint[3]
        self.x_desired[8] = self.waypoint[4]
        self.x_desired[9] = self.waypoint[4]
        self.x_desired[10] = self.waypoint[5]
        self.x_desired[11] = self.waypoint[5]
        self.x_desired[12] = 0
        self.x_desired[13] = 0
        self.x_desired[14] = 0
        self.x_desired[15] = 0
        self.x_desired[16] = 0
        self.x_desired[17] = 0


        # #Transform to body fixed frame
        # p = PoseStamped()
        # v = Vector3Stamped()
        # p.header.stamp = rospy.Time.now()
        # v.header.stamp = rospy.Time.now()
        # p.pose.position = Point(msg.x,msg.y,msg.z)
        # p.pose.orientation = Quaternion(*quaternion_from_euler(0,0,np.deg2rad(msg.yaw)))
        # v.vector = Vector3(msg.v_x,msg.v_y,msg.v_z)
        # pt = self.transform_pose(p,"hummingbird/stability_axes","world")
        # # pt_sa = self.transform_pose(p,"anafi/stability_axes","world")
        # vt = self.transform_vector3(v,"hummingbird/stability_axes","world")
        # self.x_desired = np.array([pt.pose.position.x,pt.pose.position.x,pt.pose.position.y,pt.pose.position.y,vt.vector.x,vt.vector.x,vt.vector.y,vt.vector.y,0,0,0,0])
        # self.x_state[0] = pt.pose.position.x
        # self.x_state[2] = pt.pose.position.y
        # self.x_state[4] = vt.vector.x
        # self.x_state[6] = vt.vector.y
        # self.vz_setpoint_publisher.publish(Float64(pt.pose.position.z))
        # q = pt.pose.orientation
        # (roll,pitch,yaw)= euler_from_quaternion([q.x,q.y,q.z,q.w])
        # self.yaw_pid_state_publisher.publish(Float64(0))
        # self.yaw_pid_setpoint_publisher.publish(Float64(yaw))

        # print("read waypoints")
        # print("set self.x_desired to",self.x_desired)
        return
    
    def read_vz_control_effort(self,msg):
        self.vz_control_effort = msg.data
        return
    
    def read_yaw_pid_control_effort(self,msg):
        self.yaw_control_effort = msg.data
        return


    def transform_vector3(self,vector3,target_frame_name,original_frame_name):
        print(target_frame_name)
        trans_world_to_target_frame = self.tfBuffer.lookup_transform(target_frame_name,original_frame_name, rospy.Time())
        vector3_transformed = tf2_geometry_msgs.do_transform_vector3(vector3,trans_world_to_target_frame)
        return vector3_transformed
    
    def transform_pose(self,pose,target_frame_name,original_frame_name):
        trans_world_to_target_frame = self.tfBuffer.lookup_transform(target_frame_name,original_frame_name, rospy.Time())
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose,trans_world_to_target_frame)
        return pose_transformed



    def publish_rpyt(self):
        #Perform transformation to stability axes here
        v = Vector3Stamped()
        v.header.stamp = rospy.Time.now()



        v.vector.x = self.u_trajectory[0,0] # backup 1
        v.vector.y = self.u_trajectory[0,1]# backup 1


        v.vector.z = self.x_trajectory[5,11] #backup 5



        # print("================")
        # print("u_trajectory",self.u_trajectory[1,:])
        # print("z x_trajectory",self.x_trajectory[:,4])
        # print("z x_trajectory",self.x_trajectory[:,11])
        # print("state",self.x_state)


        # print("================")

        vt = self.transform_vector3(v,drone_name+"/stability_axes","world")

        self.yaw_pid_state_publisher.publish(np.rad2deg(self.yaw_control_state))
        self.yaw_pid_setpoint_publisher.publish(Float64(self.waypoint[6]))



        # theta = np.clip(np.rad2deg(np.arcsin(vt.vector.x/9.81)),-0.5,0.5) #deg
        # phi = np.clip(np.rad2deg(np.arcsin(vt.vector.y/9.81)),-0.5,0.5) #deg
        # psi_dot = np.clip(self.yaw_control_effort,-3,3) #deg/s
        theta = np.rad2deg(np.arcsin(vt.vector.x/9.81))#deg
        phi = np.rad2deg(np.arcsin(vt.vector.y/9.81)) #deg
        psi_dot = self.yaw_control_effort #deg/s
        msg = PilotingCommand()
        msg.roll = phi #account for conversion to NED of firmware
        msg.pitch = theta #account for conversion to NED of firmware
        msg.yaw = -psi_dot 
        msg.gaz = vt.vector.z
        self.rpyt_publisher.publish(msg)
        return
    
    
    def calc_ext_force(self):
        ext_force = np.zeros((self.N,2))
        msg = Vector3Stamped()
        msg.header.frame_id = drone_name+"/stability_axes"
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = ext_force[0,0]
        msg.vector.y = ext_force[0,1]
        self.ext_force_publisher.publish(msg)
        return ext_force


if __name__ == '__main__':
    #Init node
    rospy.init_node(node_name)

    #Class initialization
    debug = MPCOSQPWaypoint()
    rate = rospy.Rate(debug.execution_freq)

    # fig_pos,axes_pos = debug.create_scalar_plot()
    # fig_F_tang,axes_F_tang = debug.create_vector_plot()
    # fig_acc,axes_acc = debug.create_debug_plot()

    print("Begin generation of solutions with desired frequency of",debug.execution_freq,"hz...")
    while not rospy.is_shutdown():
        
        
        solve_successfull = debug.solve_mpc_problem()
        if solve_successfull: 
            print("\033[95msolution found\033[0m")
            debug.update_mpc_problem()
            # print("update mpc problem worked")
            # debug.publish_motor_speed()
            # print("publish motor worked")
            debug.publish_rpyt()
        else:
            print("\033[93mCould not find solution")
            debug.update_mpc_problem()
            debug.rpyt_publisher.publish(PilotingCommand())

            # debug.publish_relative_target_pose_in_stability_axes()

 
        #Update values
        # debug.update_pos_error()

        rate.sleep()
