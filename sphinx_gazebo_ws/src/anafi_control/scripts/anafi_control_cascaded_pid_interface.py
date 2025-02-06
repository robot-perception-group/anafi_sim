import rospy
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped, QuaternionStamped, Vector3Stamped
import numpy as np
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from olympe_bridge.msg import PilotingCommand
from anafi_control.msg import State
import tf2_ros
import tf2_geometry_msgs


#Parameters
rospy.init_node('default') # node name will be overwritten by node name specified in launch file
drone_name = rospy.get_param(rospy.get_name()+'/drone_name','anafi')   
publish_hz = float(rospy.get_param(rospy.get_name()+'/publish_hz','10'))


max_roll = 30 #[deg]
max_pitch = 30 #[deg]
max_yaw_rate = 180 #[deg/s]
max_v_z = 4 #[3/s]



#define subscriber topics
#Controller related subscription topics

#Cascaded PID x-direction
#outer loop controlling p_x, output is v_x
x_position_setpoint_topic = ('/'+drone_name+'/pid/x/ol_position/setpoint_ac',Float64)
x_position_state_topic = ('/'+drone_name+'/pid/x/ol_position/state_ac',Float64)
x_position_effort_topic = ('/'+drone_name+'/pid/x/ol_position/effort_ac',Float64)
#inner loop controlling v_x, output is pitch angle
x_velocity_setpoint_topic = ('/'+drone_name+'/pid/x/il_velocity/setpoint_ac',Float64)
x_velocity_state_topic = ('/'+drone_name+'/pid/x/il_velocity/state_ac',Float64)
x_velocity_effort_topic = ('/'+drone_name+'/pid/x/il_velocity/effort_ac',Float64)

#Cascaded PID y-direction
#outer loop controlling p_y, output is v_y
y_position_setpoint_topic = ('/'+drone_name+'/pid/y/ol_position/setpoint_ac',Float64)
y_position_state_topic = ('/'+drone_name+'/pid/y/ol_position/state_ac',Float64)
y_position_effort_topic = ('/'+drone_name+'/pid/y/ol_position/effort_ac',Float64)
#inner loop controlling v_y, output is roll angle
y_velocity_setpoint_topic = ('/'+drone_name+'/pid/y/il_velocity/setpoint_ac',Float64)
y_velocity_state_topic = ('/'+drone_name+'/pid/y/il_velocity/state_ac',Float64)
y_velocity_effort_topic = ('/'+drone_name+'/pid/y/il_velocity/effort_ac',Float64)


#PID z-direction
#One loop controlling p_z, output is v_z
z_position_setpoint_topic = ('/'+drone_name+'/pid/z/setpoint_ac',Float64)
z_position_state_topic = ('/'+drone_name+'/pid/z/state_ac',Float64)
z_position_effort_topic = ('/'+drone_name+'/pid/z/effort_ac',Float64)

#PID yaw
#One loop controlling yaw angle, output is yaw_rate
yaw_setpoint_topic = ('/'+drone_name+'/pid/yaw/setpoint_ac',Float64)
yaw_state_topic = ('/'+drone_name+'/pid/yaw/state_ac',Float64)
yaw_effort_topic = ('/'+drone_name+'/pid/yaw/effort_ac',Float64)


#Waypoint topic
waypoint_topic = ('/'+drone_name+"/drone_frame/waypoint/state_ac",State)

#Drone topic
drone_topic =  ('/'+drone_name+"/drone_frame/drone/state_ac",State)

#Drone command topic
drone_command_topic = ('/'+drone_name+"/drone/rpyt",PilotingCommand)


class PIDControlInterface():
    def __init__(self):
        #Define variables
        self.piloting_command = PilotingCommand()
        self.piloting_command_stability_axes = PilotingCommand()
        self.drone_state = State()
        self.wp_state = State()

        #Define publishers
        self.drone_command_publisher = rospy.Publisher(drone_command_topic[0],drone_command_topic[1],queue_size = 0)
        self.x_position_setpoint_publisher = rospy.Publisher(x_position_setpoint_topic[0],x_position_setpoint_topic[1],queue_size=0)
        self.x_position_state_publisher = rospy.Publisher(x_position_state_topic[0],x_position_state_topic[1],queue_size=0)
        self.x_velocity_setpoint_publisher = rospy.Publisher(x_velocity_setpoint_topic[0],x_velocity_setpoint_topic[1],queue_size=0)
        self.x_velocity_state_publisher = rospy.Publisher(x_velocity_state_topic[0],x_velocity_state_topic[1],queue_size=0)
        self.y_position_setpoint_publisher = rospy.Publisher(y_position_setpoint_topic[0],y_position_setpoint_topic[1],queue_size=0)
        self.y_position_state_publisher = rospy.Publisher(y_position_state_topic[0],y_position_state_topic[1],queue_size=0)
        self.y_velocity_setpoint_publisher = rospy.Publisher(y_velocity_setpoint_topic[0],y_velocity_setpoint_topic[1],queue_size=0)
        self.y_velocity_state_publisher = rospy.Publisher(y_velocity_state_topic[0],y_velocity_state_topic[1],queue_size=0)
        self.z_position_setpoint_publisher = rospy.Publisher(z_position_setpoint_topic[0],z_position_setpoint_topic[1],queue_size=0)
        self.z_position_state_publisher = rospy.Publisher(z_position_state_topic[0],z_position_state_topic[1],queue_size=0)
        self.yaw_setpoint_publisher = rospy.Publisher(yaw_setpoint_topic[0],yaw_setpoint_topic[1],queue_size=0)
        self.yaw_state_publisher = rospy.Publisher(yaw_state_topic[0],yaw_state_topic[1],queue_size=0)

        #Define subscribers
        #Outer loop / single loop efforts
        self.x_position_effort_subscriber = rospy.Subscriber(x_position_effort_topic[0],x_position_effort_topic[1],self.read_x_position_effort)
        self.y_position_effort_subscriber = rospy.Subscriber(y_position_effort_topic[0],y_position_effort_topic[1],self.read_y_position_effort)
        self.z_position_effort_subscriber = rospy.Subscriber(z_position_effort_topic[0],z_position_effort_topic[1],self.read_z_position_effort)
        self.yaw_effort_effort_subscriber = rospy.Subscriber(yaw_effort_topic[0],yaw_effort_topic[1],self.read_yaw_effort)
        #Inner loop efforts
        self.x_velocity_effort_effort_subscriber = rospy.Subscriber(x_velocity_effort_topic[0],x_velocity_effort_topic[1],self.read_x_velocity_effort)
        self.y_velocity_effort_effort_subscriber = rospy.Subscriber(y_velocity_effort_topic[0],y_velocity_effort_topic[1],self.read_y_velocity_effort)
        #Other subscribers
        self.wp_state_subscriber = rospy.Subscriber(waypoint_topic[0],waypoint_topic[1],self.read_target_state)
        self.drone_state_subscriber = rospy.Subscriber(drone_topic[0],drone_topic[1],self.read_drone_state)

        #Transformations
        #Init tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.wp_frame = 'world'
        # self.drone_frame = drone_name+'/stability_axes'
        self.drone_frame = drone_name+'/stability_axes'

        #Variables
        self.drone_state = State()
        self.target_state = State()
        self.piloting_command = PilotingCommand()
        self.piloting_command_stability_axes = PilotingCommand()
        return

    
    def read_x_position_effort(self,msg):
        #send effort from outer loop as setpoint to inner loop
        self.x_velocity_setpoint_publisher.publish(msg)
        return
    
    def read_y_position_effort(self,msg):
        #send effort from outer loop as setpoint to inner loop
        self.y_velocity_setpoint_publisher.publish(msg)
        return
    
    def read_z_position_effort(self,msg):
        self.piloting_command.gaz = msg.data
        return
    
    def read_yaw_effort(self,msg):
        self.piloting_command.yaw = msg.data
        return
    
    def read_x_velocity_effort(self,msg):
        self.piloting_command.pitch = np.deg2rad(msg.data)
        self.piloting_command.pitch = msg.data
        return
    
    def read_y_velocity_effort(self,msg):
        self.piloting_command.roll = msg.data

        return
    
    def read_target_state(self,msg):
        self.target_state = msg
        self.x_position_setpoint_publisher.publish(Float64(msg.pose.pose.position.x))
        self.y_position_setpoint_publisher.publish(Float64(msg.pose.pose.position.y))
        self.z_position_setpoint_publisher.publish(Float64(msg.pose.pose.position.z))
        q = msg.pose.pose.orientation
        (roll,pitch,yaw) = euler_from_quaternion((q.x,q.y,q.z,q.w))
        self.yaw_setpoint_publisher.publish(Float64(np.rad2deg(yaw)))

        return
    
    def read_drone_state(self,msg):
        self.drone_state = msg
        self.x_position_state_publisher.publish(Float64(msg.pose.pose.position.x))
        self.y_position_state_publisher.publish(Float64(msg.pose.pose.position.y))
        self.z_position_state_publisher.publish(Float64(msg.pose.pose.position.z))

        self.x_velocity_state_publisher.publish(Float64(msg.twist.twist.linear.vector.x))
        self.y_velocity_state_publisher.publish(Float64(msg.twist.twist.linear.vector.y))
        q = msg.pose.pose.orientation
        (roll,pitch,yaw) = euler_from_quaternion((q.x,q.y,q.z,q.w))
        self.yaw_state_publisher.publish(Float64(np.rad2deg(yaw)))

        return
    
    def transform_piloting_command_to_drone_frame(self):
        rpy_command_world = PoseStamped()
        rpy_command_world.header.stamp = rospy.Time.now()
        rpy_command_world.header.frame_id = "world"
        drone_roll,drone_pitch,drone_yaw = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])
        q_pc = quaternion_from_euler(self.piloting_command.roll,self.piloting_command.pitch,drone_yaw)

        rpy_command_world.pose.orientation.x = q_pc[0]
        rpy_command_world.pose.orientation.y = q_pc[1]
        rpy_command_world.pose.orientation.z = q_pc[2]
        rpy_command_world.pose.orientation.w = q_pc[3]
        rpy_command_stability_axes = PoseStamped()
        try:
            trans_world_to_drone_frame = self.tfBuffer.lookup_transform(self.drone_frame,self.wp_frame, rospy.Time())
            rpy_command_stability_axes = tf2_geometry_msgs.do_transform_pose(rpy_command_world,trans_world_to_drone_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('transformation from wp_frame to drone_frame threw an error')

        q_p_c_sa = rpy_command_stability_axes.pose.orientation
        roll_sa,pitch_sa,yaw_sa = euler_from_quaternion([q_p_c_sa.x,q_p_c_sa.y,q_p_c_sa.z,q_p_c_sa.w])

        
        self.piloting_command_stability_axes.roll = np.rad2deg(roll_sa)
        self.piloting_command_stability_axes.pitch =  np.rad2deg(pitch_sa)
        self.piloting_command_stability_axes.yaw = self.piloting_command.yaw
        self.piloting_command_stability_axes.gaz = self.piloting_command.gaz
        return


    def configure_piloting_command(self,pcmd):

        piloting_command_configured = pcmd
        piloting_command_configured.roll = np.rad2deg(pcmd.roll)
        piloting_command_configured.pitch = np.rad2deg(pcmd.pitch)
        return piloting_command_configured




if __name__ == '__main__':

    pid_interface = PIDControlInterface()
    
   
    #Init rate
    rate = rospy.Rate(publish_hz)

    #commands to be executed as long as node is up
    while not rospy.is_shutdown():

        pid_interface.piloting_command.header.stamp = rospy.Time.now()
        pid_interface.drone_command_publisher.publish(pid_interface.piloting_command) # Have you selected the correct drone state topic?
        rate.sleep()