#!/home/raven/venvs/sphinx_with_gazebo/bin/python

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



#Parameters
node_name ='trajectory_generator_node' 
drone_name = rospy.get_param(rospy.get_name()+'/drone_name','anafi')   
publish_hz = float(rospy.get_param(rospy.get_name()+'/publish_hz','10'))
prefix = "/trajectory_generator"

#Define topics
waypoints_topic = ('/'+drone_name+'/position_control/waypoint',Waypoint)
static_waypoint_topic = (prefix+'/static_waypoint',Waypoint)
trajectory_type_modifier_topic = (prefix+'/trajectory_type_modifer',String)
trajectory_speed_longitudinal_topic = (prefix+'/setpoints/v_x',Float64)
trajectory_speed_lateral_topic      = (prefix+'/setpoints/v_y',Float64)
trajectory_speed_vertical_topic     = (prefix+'/setpoints/v_z',Float64)
trajectory_radius_longitudinal_topic = (prefix+'/setpoints/r_x',Float64)
trajectory_radius_lateral_topic = (prefix+'/setpoints/r_y',Float64)
trajectory_radius_vertical_topic = (prefix+'/setpoints/r_z',Float64)
trajectory_begin_timer_topic = (prefix+'/begin',Bool)
moveto_topic = ('/'+drone_name+'/drone/moveto_debug',MoveToCommand)

class TrajectoryGenerator:
    """Class that creates a trajectory and publishes it using geometry_msgs/Pose messages"""

    def __init__(self):
        """Class contains data required in order to update the current position of the moving platform."""        
        #Get parameters

        self.trajectory_start_position = rospy.get_param(rospy.get_name()+"/trajectory_start_position",{'x':0,'y':0,'z':10}) #[m]
        self.trajectory_start_orientation = rospy.get_param(rospy.get_name()+"/trajectory_start_orientation",{'phi':0,'theta':0,'psi':0}) #{rad}


        self.trajectory_radius_longitudinal = float(rospy.get_param(rospy.get_name()+"/r_x","10")) #[m]
        self.trajectory_speed_longitudinal = float(rospy.get_param(rospy.get_name()+"/v_x","1")) #[m/s]
        
        self.trajectory_radius_lateral = float(rospy.get_param(rospy.get_name()+"/r_y","10")) #[m]
        self.trajectory_speed_lateral = float(rospy.get_param(rospy.get_name()+"/v_y","1")) #[m/s]

        self.trajectory_type_vertical = rospy.get_param(rospy.get_name()+"/trajectory_type_z","straight")
        self.trajectory_radius_vertical = float(rospy.get_param(rospy.get_name()+"/r_z","10")) #[m]
        self.trajectory_speed_vertical = float(rospy.get_param(rospy.get_name()+"/v_z","1")) #[m/s]

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
        self.delta_t = 1 / publish_hz #sec  


        #Publishers
        self.waypoints_publisher = rospy.Publisher(waypoints_topic[0],waypoints_topic[1],queue_size = 1)
        self.timer_publisher = rospy.Publisher(trajectory_begin_timer_topic[0],trajectory_begin_timer_topic[1],queue_size=1)
        self.moveto_publisher = rospy.Publisher(moveto_topic[0],moveto_topic[1],queue_size=1)
        
        self.trajectory_type_modifer_subscriber = rospy.Subscriber(trajectory_type_modifier_topic[0],trajectory_type_modifier_topic[1],self.read_trajectory_type_modifier)
        self.trajectory_speed_longitudinal_subscriber = rospy.Subscriber(trajectory_speed_longitudinal_topic[0],trajectory_speed_longitudinal_topic[1],self.read_trajectory_speed_longitudinal)
        self.trajectory_radius_longitudinal_subscriber = rospy.Subscriber(trajectory_radius_longitudinal_topic[0],trajectory_radius_longitudinal_topic[1],self.read_trajectory_radius_longitudinal)
        self.trajectory_speed_lateral_subscriber = rospy.Subscriber(trajectory_speed_lateral_topic[0],trajectory_speed_lateral_topic[1],self.read_trajectory_speed_lateral)
        self.trajectory_radius_lateral_subscriber = rospy.Subscriber(trajectory_radius_lateral_topic[0],trajectory_radius_lateral_topic[1],self.read_trajectory_radius_lateral)
        self.trajectory_speed_vertical_subscriber = rospy.Subscriber(trajectory_radius_vertical_topic[0],trajectory_radius_vertical_topic[1],self.read_trajectory_speed_vertical)
        self.trajectory_radius_vertical_subscriber = rospy.Subscriber(trajectory_radius_vertical_topic[0],trajectory_radius_vertical_topic[1],self.read_trajectory_radius_vertical)
        self.waypoint_subscriber = rospy.Subscriber(static_waypoint_topic[0],static_waypoint_topic[1],self.read_static_waypoint)
        #variables
        self.v_x_wp = 0
        self.v_y_wp = 0
        self.v_z_wp = 0
        self.p_x_wp = self.trajectory_start_position['x']
        self.p_y_wp = self.trajectory_start_position['y']
        self.p_z_wp = self.trajectory_start_position['z']
        self.r_x = 3
        self.r_y = 3
        self.r_z = 2
        self.v_x = 0
        self.v_y = 0
        self.v_z = 2
        self.static_waypoint = Waypoint()
        self.trajectory_type_modifier = "static"
        return

    def read_static_waypoint(self,msg):
        self.static_waypoint = msg
        return

    def read_trajectory_speed_longitudinal(self,msg):
        """Function reads the commanded trajectory speed for longitudinal motion from the associated ROS topic."""
        self.v_x = msg.data
        return

    def read_trajectory_radius_longitudinal(self,msg):
        """Function reads the commanded trajectory radius for longitudinal motion from the associated ROS topic."""
        self.r_x = msg.data
        return

    def read_trajectory_speed_lateral(self,msg):
        """Function reads the commanded trajectory speed for the lateral motion from the associated ROS topic."""
        self.v_y = msg.data
        return

    def read_trajectory_radius_lateral(self,msg):
        """Function reads the commanded trajectory radius for the lateral motion from the associated ROS topic."""
        self.r_y = msg.data
        return
    
    def read_trajectory_speed_vertical(self,msg):
        """Function reads the commanded trajectory speed for the vertical motion from the associated ROS topic."""
        self.v_z = msg.data
        return

    def read_trajectory_radius_vertical(self,msg):
        """Function reads the commanded trajectory radius for the vertical motion from the associated ROS topic."""
        self.r_z = msg.data
        return
    
    def read_trajectory_type_modifier(self,msg):
        """Function reads the requested trajectory type update from the associated ROS topic."""
        self.trajectory_type_modifier = msg.data
        return
    
    


    def compute_trajectory_circle(self,active_axis_0,active_axis_1,inactive_axis):
        """Function computes a circular trajectory for the moving platform."""

        omega = self.v_x / self.r_x
        if not np.isclose(self.v_x,self.v_y):
            print("The velocity of axis 0 is not equal to the velocity of axis 1. Calculating angular frequency of circular trajectory using the velocity of axis 0.")
        x = getattr(self,"r_"+active_axis_0)*np.cos(omega*self.t) + self.trajectory_start_position[active_axis_0]
        y = getattr(self,"r_"+active_axis_1)*np.sin(omega*self.t)+ self.trajectory_start_position[active_axis_1]
        u = getattr(self,"r_"+active_axis_0)*omega*(-np.sin(omega*self.t))
        v = getattr(self,"r_"+active_axis_1)*omega*(np.cos(omega*self.t))
        # print("a_x =",getattr(self,"r_"+active_axis_0)*omega*omega*(-np.cos(omega*self.t)))
        # print("a_y =",getattr(self,"r_"+active_axis_1)*omega*omega*(-np.sin(omega*self.t)))

        setattr(self,"p_"+active_axis_0+"_wp",x)
        setattr(self,"p_"+active_axis_1+"_wp",y)
        setattr(self,"v_"+active_axis_0+"_wp",u)
        setattr(self,"v_"+active_axis_1+"_wp",v)
        return

    def compute_trajectory_straight(self,active_axes_list):
        """Function computes a straight trajectory for the moving platform."""
        # self.x,self.y,self.z = 1,1,1
        # self.v_x,self.v_y,self.v_z = 0,0,0
        if "x" in active_axes_list:
            self.p_x_wp = self.p_x_wp + self.v_x * self.delta_t
            self.v_x_wp = self.v_x
        if "y" in active_axes_list:
            self.p_y_wp = self.p_y_wp + self.v_y * self.delta_t
            self.v_y_wp = self.v_y
        if "z" in active_axes_list:
            self.p_z_wp = self.p_z_wp + self.v_z * self.delta_t
            self.v_z_wp = self.v_z
        return 
    
    def compute_static_trajectory(self):
        self.p_x_wp = self.static_waypoint.x
        self.p_y_wp = self.static_waypoint.y
        self.p_z_wp = self.static_waypoint.z
        self.v_x_wp = 0 # no velocity because it is static
        self.v_y_wp = 0 # no velocity because it is static
        self.v_z_wp = 0 # no velocity because it is static
        return


    def compute_trajectory_rectiliar_periodic_straight(self,active_axes_list):
        """Function computes a rectiliar periodic trajectory for the moving platform."""
        # self.x,self.y,self.z = 1,1,1
        # self.v_x,self.v_y,self.v_z = 0,0,0

        if "x" in active_axes_list:
            omega_x = self.v_x / self.r_x
            self.p_x_wp = self.r_x*np.sin(omega_x*self.t) + self.trajectory_start_position["x"]
            self.v_x_wp = self.r_x*omega_x*np.cos(omega_x*self.t)
        if "y" in active_axes_list:
            omega_y = self.v_y / self.r_y
            self.p_y_wp = self.r_y*np.sin(omega_y*self.t) + self.trajectory_start_position["y"]
            self.v_y_wp = self.r_y*omega_y*np.cos(omega_y*self.t)
        if "z" in active_axes_list:
            omega_z = self.v_z / self.r_z
            self.p_z_wp = self.r_z*np.sin(omega_z*self.t) + self.trajectory_start_position["z"]
            self.v_z_wp = self.r_z*omega_z*np.cos(omega_z*self.t)
        return 


    def trajectory_type_modifier_function(self):
        """Function computes selects the functions to compute the desired composed trajectory.
        Format: trajectoryIDhorizontal_InfoHorizontal:trajectoryIDvertical_InfoVertical
            trajectoryIDhorizontal  :   Must be either "circle", "straight", "rectilinear"
            trajectoryIDvertical    :   Must be either "circle", "straight", "rectilinear"
            InfoHorizontal          :   List of axis, i.e. either "x","y","xy", if trajectoryIDhorizontal is either "straight" or "rectilinear"
                                        activeAxis0activeAxis1_inactiveAxis (e.g. "xy_z"), if trajectoryIDhorizontal is circle
            InfoVertical          :     List of axis, i.e. either,"z","xz","yz", if trajectoryIDvertical is either "straight" or "rectilinear"
                                        activeAxis0activeAxis1_inactiveAxis (e.g. "xz_y"), if trajectoryIDhorizontal is circle                                        
            """


        print("Requested trajectory type:", self.trajectory_type_modifier)
        traj_type_split = self.trajectory_type_modifier.split(":") #separate in horizontal and vertical movement
        #Handle static case
        if "static" in self.trajectory_type_modifier:
            self.compute_static_trajectory()
            return

        #Handle horizontal motion
        split_horiz = traj_type_split[0].split("_")
        if "circle" in split_horiz[0]:
            self.compute_trajectory_circle(split_horiz[1][0],split_horiz[1][1],split_horiz[2])
        if "straight" in split_horiz[0]:
            self.compute_trajectory_straight(split_horiz[1])
        if "rectilinear" in split_horiz[0]:
            self.compute_trajectory_rectiliar_periodic_straight(split_horiz[1])
        #Handle vertical motion
        split_vert = traj_type_split[1].split("_")

        if "circle" in split_vert[0]:
            self.compute_trajectory_circle(split_vert[1][0],split_vert[1][1],split_vert[2])
        if "straight" in split_vert[0]:
            self.compute_trajectory_straight(split_vert[1])
        if "rectilinear" in split_vert[0]:
            self.compute_trajectory_rectiliar_periodic_straight(split_vert[1])
        return


    def publish_trajectory(self):
        """Function publishes the updated platform position to the ROS network."""
        msg = Waypoint()
        msg.x = self.p_x_wp
        msg.y = self.p_y_wp
        msg.z = self.p_z_wp
        msg.v_x = self.v_x_wp
        msg.v_y = self.v_y_wp
        msg.v_z = self.v_z_wp   
        self.waypoints_publisher.publish(msg)
        print("self.p_z_wp,",self.p_z_wp)


        #Parrot headquarter in Paris is the default home location
        lat0 = 48.878922
        lon0 = 2.367782
        h0 = 0

        lat1,lon1,alt1 = pymap3d.enu2geodetic(self.p_x_wp,self.p_y_wp,self.p_z_wp,lat0,lon0,h0,deg =True)
        msg_moveto = MoveToCommand()
        msg_moveto.header.stamp = rospy.Time.now()
        msg_moveto.latitude = lat1
        msg_moveto.longitude = lon1
        msg_moveto.altitude = alt1
        msg_moveto.heading = 90
        msg_moveto.orientation_mode = 2
        self.moveto_publisher.publish(msg_moveto)




        return



if __name__ == '__main__':
    rospy.init_node(node_name, anonymous=True)
    trajectory_generator = TrajectoryGenerator()
    
    rate = rospy.Rate(publish_hz)
    while not rospy.is_shutdown():
        trajectory_generator.trajectory_type_modifier_function()
        trajectory_generator.publish_trajectory()
        trajectory_generator.t += trajectory_generator.delta_t
        omega_x = trajectory_generator.v_x / trajectory_generator.r_x
        omega_y = trajectory_generator.v_y / trajectory_generator.r_y
        omega_z = trajectory_generator.v_z / trajectory_generator.r_z
        print("r_x:",trajectory_generator.r_x)
        print("r_y:",trajectory_generator.r_y)
        print("r_z:",trajectory_generator.r_z)
        print("v_x:",trajectory_generator.v_x)
        print("v_y:",trajectory_generator.v_y)
        print("v_z:",trajectory_generator.v_z)

        print("omega_x =",omega_x)
        if trajectory_generator.t > 2*np.pi/omega_z:
            trajectory_generator.timer_publisher.publish(True)
            trajectory_generator.t = 0
        rate.sleep()
