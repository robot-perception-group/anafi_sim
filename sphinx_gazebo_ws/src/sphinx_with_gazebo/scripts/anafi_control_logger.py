# [ ] Check if header matches logged data
# [X] Check if xyz components are selected properly in read functions
# [X] Check if xyz components are selected properly in logging sections
# [ ] Check if init of blimp works as expected
# [X] Check total duration in logged files
# 

import yaml
import rospy
from std_msgs.msg import Bool,Float64MultiArray,Header
from geometry_msgs.msg import TransformStamped, Point,Pose, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
from copy import deepcopy
import rospkg
import os
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from std_srvs.srv import Empty
from sphinx_with_gazebo.utils import write_data_to_csv, create_log_dir_path, create_log_dir, print_progress_bar
from anafi_control.msg import State
from anafi_control.msg import Waypoint
from sphinx_with_gazebo.msg import Sphinx





#read gust params
rospack = rospkg.RosPack()
max_number_episodes = 30
log_file_path = "/home/pgoldschmid/src/sphinx_with_gazebo_code/experiments_evaluation/sim_data"

#read launch params
node_name = "sphinx_logger_node"
rospy.init_node(node_name) # node name will be overwritten by node name specified in launch file
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name',"anafi")
publish_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/publish_hz',100))

uav_state_anafi_topic = ("/"+drone_name+"/position_control/state_enu",State)
uav_state_sphinx_topic = ("/"+drone_name+"/sphinx/drone_data",Sphinx)


waypoint_topic =  ('/'+drone_name+'/position_control/waypoint',Waypoint)
trajectory_begin_timer_topic = ('/trajectory_generator'+'/begin',Bool)

#publisher topics
activate_wind_gust_topic = ("/gazebo/ActivateWindGustGenerator",Bool)

class GustDockingRecorder():
    def __init__(self):
        #variables
        self.copter_pos_x_sphinx = 0
        self.copter_pos_y_sphinx = 0
        self.copter_pos_z_sphinx = 0
        self.copter_roll_sphinx =  0
        self.copter_pitch_sphinx = 0
        self.copter_yaw_sphinx = 0
        self.copter_vel_x_sphinx = 0
        self.copter_vel_y_sphinx = 0
        self.copter_vel_z_sphinx = 0
        self.wp_x = 0
        self.wp_y = 0
        self.wp_z = 0
        self.wp_vx = 0
        self.wp_vy = 0
        self.wp_vz = 0
        self.wp_yaw = 0

        self.header = Header()

        self.recording_active = False
        self.publish_hz = publish_hz
        self.log_file_path = log_file_path




        #subscriber 
        self.uav_state_anafi_subscriber = rospy.Subscriber(uav_state_anafi_topic[0],uav_state_anafi_topic[1],self.read_uav_state_anafi)
        self.uav_state_sphinx_subscriber = rospy.Subscriber(uav_state_sphinx_topic[0],uav_state_sphinx_topic[1],self.read_uav_state_sphinx)
        self.waypoint_subscriber = rospy.Subscriber(waypoint_topic[0],waypoint_topic[1],self.read_waypoint)
        self.trajectory_begin_timer_subscriber = rospy.Subscriber(trajectory_begin_timer_topic[0],trajectory_begin_timer_topic[1],self.read_begin_timer)

        #publisher        
        self.activate_wind_gust_generator_publisher = rospy.Publisher(activate_wind_gust_topic[0],activate_wind_gust_topic[1],queue_size = 1)

        #Flags and increments
        self.begin_flag = False
        self.logging_active = False
        self.node_shutdown_requested = False
        self.episode_completed = False
        self.i = -1

        #Print additional information
        return
    
    def convert_quat_to_euler(self,quat):
        q = [quat.x, quat.y, quat.z, quat.w]
        (r,p,y) = euler_from_quaternion(q)
        return r,p,y


    def episode_manager(self):

        #Update progress bar
        print_progress_bar(self.i,max_number_episodes,prefix="Tot. "+str(max_number_episodes)+" episodes",suffix="complete",length=50)
        #Update episode counter and check terminal criterion
        # self.i += 1
        if self.i >= max_number_episodes: 
            print("Maximum number of episodes reached")
            self.node_shutdown_requested = True 
        return


    
    def convert_float_type(self,row:list,desired_float_type:str):
        if desired_float_type == "float16":
            updated_dtype_list = list()
            #Iterate through each element in list
            for item in row:
                if isinstance(item,float):
                    updated_dtype_list.append(np.float16(item)) 
                else:
                    updated_dtype_list.append(item)

        elif desired_float_type == "float32":
            updated_dtype_list = list()
            #Iterate through each element in list
            for item in row:
                if isinstance(item,float):
                    updated_dtype_list.append(np.float32(item)) 
                else:
                    updated_dtype_list.append(item)

        elif desired_float_type == "float64":
            updated_dtype_list = list()
            #Iterate through each element in list
            for item in row:
                if isinstance(item,float):
                    updated_dtype_list.append(np.float64(item)) 
                else:
                    updated_dtype_list.append(item)

        else:
            raise NotImplementedError
        return updated_dtype_list
    

    def log_response(self):
        '''
        Function performs the logging
        '''
        log_file_path = os.path.join(self.log_file_path,"episode_"+str(self.i)+".csv")

        #If log file does not exist yet, create header
        if not os.path.isfile(log_file_path):
            header_row = []
            #General data
            header_row += ["time"]


            #Blimp data
            header_row += ["x_sphinx"   ,"y_sphinx"    ,"z_sphinx"]
            header_row += ["vx_sphinx"  ,"vy_sphinx"   ,"vz_sphinx"]
            header_row += ["roll_sphinx","pitch_sphinx","yaw_sphinx"]
            header_row += ["x_anafi"   ,"y_anafi"    ,"z_anafi"]
            header_row += ["vx_anafi"  ,"vy_anafi"   ,"vz_anafi"]
            header_row += ["roll_anafi","pitch_anafi","yaw_anafi"]
            header_row += ["wp_x"   ,"wp_y"    ,"wp_z"]
            header_row += ["wp_vx"  ,"wp_vy"   ,"wp_vz"]
            header_row += ["wp_yaw"]


            
            write_data_to_csv(log_file_path,header_row)
        else:
            #If log file already exists
            new_row = []
            #General data
            new_row += [rospy.Time.now()]
            #Blimp data
            new_row += [self.copter_pos_x_sphinx,self.copter_pos_y_sphinx,self.copter_pos_z_sphinx]
            new_row += [self.copter_vel_x_sphinx,self.copter_vel_y_sphinx,self.copter_vel_z_sphinx]
            new_row += [self.copter_roll_sphinx,self.copter_pitch_sphinx,self.copter_yaw_sphinx]
            new_row += [self.copter_pos_x_anafi,self.copter_pos_y_anafi,self.copter_pos_z_anafi]
            new_row += [self.copter_vel_x_anafi,self.copter_vel_y_anafi,self.copter_vel_z_anafi]
            new_row += [ self.copter_roll_anafi,self.copter_pitch_anafi,  self.copter_yaw_anafi]
            new_row += [self.wp_x,self.wp_y,self.wp_z]
            new_row += [self.wp_vx,self.wp_vy,self.wp_vz]
            new_row += [self.wp_yaw]
            dtype_new_row = self.convert_float_type(new_row,"float16")
            # print("wrote")
            # print(dtype_new_row)

            write_data_to_csv(log_file_path,dtype_new_row)
        return
    


    def read_uav_state_anafi(self,msg):
        
        self.copter_pos_x_anafi = msg.pose.pose.position.x
        self.copter_pos_y_anafi = msg.pose.pose.position.y
        self.copter_pos_z_anafi = msg.pose.pose.position.z
        roll,pitch,yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.copter_roll_anafi = roll
        self.copter_pitch_anafi = pitch
        self.copter_yaw_anafi = yaw
        self.copter_vel_x_anafi = msg.twist.twist.linear.vector.x
        self.copter_vel_y_anafi = msg.twist.twist.linear.vector.y
        self.copter_vel_z_anafi = msg.twist.twist.linear.vector.z
        return
    
    def read_uav_state_sphinx(self,msg):
        
        self.copter_pos_x_sphinx = msg.posX
        self.copter_pos_y_sphinx = msg.posY
        self.copter_pos_z_sphinx = msg.posZ
        self.copter_roll_sphinx  = msg.attitudeX
        self.copter_pitch_sphinx = msg.attitudeY
        self.copter_yaw_sphinx   = msg.attitudeZ
        self.copter_vel_x_sphinx = msg.velXENU
        self.copter_vel_y_sphinx = msg.velYENU
        self.copter_vel_z_sphinx = msg.velZENU
        return
    
    
    def read_waypoint(self,msg):
         #Store data
        self.header.stamp = rospy.Time.now()
        self.wp_x = msg.x
        self.wp_y = msg.y
        self.wp_z = msg.z
        self.wp_vx = msg.v_x
        self.wp_vy = msg.v_y
        self.wp_vz = msg.v_z
        self.wp_yaw = msg.yaw
        return
    
    def read_begin_timer(self,msg):
        if msg.data == True and not self.begin_flag:
            self.i += 1 
            # self.log_response()
            self.logging_active = True
        if msg.data == True and self.begin_flag and self.logging_active:
            self.logging_active = False
            self.begin_flag = False
            print("Deactivating logging...")

        
        return
            

if __name__ == '__main__':

    gust_response_recorder = GustDockingRecorder()
   
    #Init rate
    # rate = rospy.Rate(gust_response_recorder.publish_hz)
    # rate = rospy.Rate(1)
    rate = rospy.Rate(publish_hz)

    print_progress_bar(0,max_number_episodes,prefix="Tot. "+str(max_number_episodes)+" episodes",suffix="complete",length=50)
    #commands to be executed as long as node is up
    while not rospy.is_shutdown():
        gust_response_recorder.episode_manager()
        if gust_response_recorder.logging_active:
            gust_response_recorder.log_response()
            # print("logging active")
        if gust_response_recorder.node_shutdown_requested:
            print("Shutting down")
            rospy.signal_shutdown("Maximum number of episodes reached. Shutdown initiated...")
        rate.sleep()
