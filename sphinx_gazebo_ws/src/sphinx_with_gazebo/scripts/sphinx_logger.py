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
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelStates, ModelState
from sphinx_with_gazebo.utils import write_data_to_csv, create_log_dir_path, create_log_dir, print_progress_bar
from docking.msg import LandingSimulationObjectState





#read gust params
rospack = rospkg.RosPack()


#read launch params
rospy.init_node('default') # node name will be overwritten by node name specified in launch file

#subscriber topics
wind_gust_information_topic = ("/gazebo/WindGustInfo",Float64MultiArray)
sa_copter_topic = ("/hummingbird/landing_simulation/drone/state",LandingSimulationObjectState)
sa_blimp_topic = ("/hummingbird/landing_simulation/moving_platform/state",LandingSimulationObjectState)

uav_state_topic = ('/hummingbird/landing_simulation/world_frame/drone/state',LandingSimulationObjectState)
mp_state_topic =  ('/hummingbird/landing_simulation/world_frame/moving_platform/state',LandingSimulationObjectState)

#publisher topics
activate_wind_gust_topic = ("/gazebo/ActivateWindGustGenerator",Bool)

class GustDockingRecorder():
    def __init__(self):
        #params
        with open(os.path.join(rospack.get_path('docking'),'gust_response_dataset_generation'+'/params_gust_docking_recorder.yaml')) as f:
            self.params = yaml.safe_load(f)
        self.drone_name = self.params["logging"]["drone_name"]
        # --- Variables for logging ---
        #General
        self.header = Header()
        #Gazebo blimp data
        self.blimp_pose_gazebo = Pose()
        self.blimp_twist_gazebo = Twist()
        self.blimp_roll_gazebo = 0
        self.blimp_pitch_gazebo = 0
        self.blimp_yaw_gazebo = 0

        self.blimp_pose_sa = Pose()
        self.blimp_twist_sa = Twist()
        self.blimp_roll_sa = 0
        self.blimp_pitch_sa = 0
        self.blimp_yaw_sa = 0
        #Gazebo copter data
        self.copter_pose_gazebo = Pose()
        self.copter_twist_gazebo = Twist()
        self.copter_roll_gazebo = 0
        self.copter_pitch_gazebo = 0
        self.copter_yaw_gazebo = 0
        
        self.copter_pose_sa = Pose()
        self.copter_twist_sa = Twist()
        self.copter_roll_sa = 0
        self.copter_pitch_sa = 0
        self.copter_yaw_sa = 0

        #nn coeffs
        self.sa_ax = 0
        self.sa_bx = 0
        self.sa_cx = 0
        self.sa_dx = 0
        self.sa_ay = 0
        self.sa_by = 0
        self.sa_cy = 0
        self.sa_dy = 0
        self.sa_az = 0
        self.sa_bz = 0
        self.sa_cz = 0
        self.sa_dz = 0        
        self.enu_ax = 0
        self.enu_bx = 0
        self.enu_cx = 0
        self.enu_dx = 0
        self.enu_ay = 0
        self.enu_by = 0
        self.enu_cy = 0
        self.enu_dy = 0
        self.enu_az = 0
        self.enu_bz = 0
        self.enu_cz = 0
        self.enu_dz = 0

        #Gust information
        #Directional information
        self.normalized_gust_direction = Point()
        self.cur_gust_velocity = Point()
        #Duration information
        self.gust_duration = 0
        self.gust_break_duration = 0
        self.pre_gust_duration = 0
        self.episode_duration = self.params["logging"]["episode_duration"]
        #Velocity information
        self.max_abs_gust_velocity = 0
        self.abs_gust_velocity = 0
        #Flags
        self.gust_status = 0


        #--- Variables for running the script ---
        self.publish_hz = self.params["logging"]["freq"]
        self.max_number_episodes = self.params["logging"]["max_number_episodes"]
        self.episode_completed = False
        self.log_dir = self.params["logging"]["log_dir"]
        self.log_id = self.params["logging"]["log_id"]
        self.log_verbose = self.params["logging"]["verbose"]

        #Create the logging directory
        self.log_file_path = create_log_dir_path(self.log_dir,self.log_id)
        create_log_dir(self.log_file_path)

        #Init data
        self.blimp_init_px = self.params["blimp_init_state"]["px"]
        self.blimp_init_py = self.params["blimp_init_state"]["py"]
        self.blimp_init_pz = self.params["blimp_init_state"]["pz"]
        self.blimp_init_vx = self.params["blimp_init_state"]["vx"]
        self.blimp_init_vy = self.params["blimp_init_state"]["vy"]
        self.blimp_init_vz = self.params["blimp_init_state"]["vz"]
        self.blimp_init_roll = self.params["blimp_init_state"]["roll"]
        self.blimp_init_pitch = self.params["blimp_init_state"]["pitch"]
        self.blimp_init_yaw = self.params["blimp_init_state"]["yaw"]

        self.copter_init_px = self.params["copter_init_state"]["px"]
        self.copter_init_py = self.params["copter_init_state"]["py"]
        self.copter_init_pz = self.params["copter_init_state"]["pz"]
        self.copter_init_vx = self.params["copter_init_state"]["vx"]
        self.copter_init_vy = self.params["copter_init_state"]["vy"]
        self.copter_init_vz = self.params["copter_init_state"]["vz"]
        self.copter_init_roll = self.params["copter_init_state"]["roll"]
        self.copter_init_pitch = self.params["copter_init_state"]["pitch"]
        self.copter_init_yaw = self.params["copter_init_state"]["yaw"]

        #Set up services
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world_gazebo_service = rospy.ServiceProxy('/gazebo/reset_world',Empty)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        rospy.wait_for_service('/gazebo/pause_physics')
        self.pause_sim = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        rospy.wait_for_service('/gazebo/get_model_state')
        self.model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)

        #subscriber 
        self.wind_gust_information_subscriber = rospy.Subscriber(wind_gust_information_topic[0],wind_gust_information_topic[1],self.read_wind_gust_information)
        # self.gazebo_model_state_subscriber = rospy.Subscriber(gazebo_model_state_topic[0],gazebo_model_state_topic[1],self.read_gazebo_model_states)
        # self.nn_coeffs_enu_subscriber = rospy.Subscriber(nn_coeffs_enu_topic[0],nn_coeffs_enu_topic[1],self.read_nn_coeffs_enu)
        # self.nn_coeffs_sa_subscriber = rospy.Subscriber(nn_coeffs_sa_topic[0],nn_coeffs_sa_topic[1],self.read_nn_coeffs_sa)
        self.copter_state_sa_subscriber = rospy.Subscriber(sa_copter_topic[0],sa_copter_topic[1],self.read_copter_state_sa)
        self.copter_state_blimp_subscriber = rospy.Subscriber(sa_blimp_topic[0],sa_blimp_topic[1],self.read_blimp_state_sa)
        self.uav_state_subscriber = rospy.Subscriber(uav_state_topic[0],uav_state_topic[1],self.read_uav_state)
        self.mp_state_subscriber = rospy.Subscriber(mp_state_topic[0],mp_state_topic[1],self.read_mp_state)

        #publisher        
        self.activate_wind_gust_generator_publisher = rospy.Publisher(activate_wind_gust_topic[0],activate_wind_gust_topic[1],queue_size = 1)

        #Flags and increments
        self.logging_active = False
        # self.episode_active = False
        self.node_shutdown_requested = False
        self.episode_completed = False
        self.i = -1

        #Print additional information
        print("Converting float type:",self.params["logging"]["cast_float_type"])
        return
    
    def convert_quat_to_euler(self,quat):
        q = [quat.x, quat.y, quat.z, quat.w]
        (r,p,y) = euler_from_quaternion(q)
        return r,p,y
    
    def read_copter_state_sa(self,msg):
        self.copter_pose_sa.position.x = msg.pose.pose.position.x
        self.copter_pose_sa.position.y = msg.pose.pose.position.y
        self.copter_pose_sa.position.z = msg.pose.pose.position.z
        self.copter_pose_sa.orientation.x = msg.pose.pose.orientation.x
        self.copter_pose_sa.orientation.y = msg.pose.pose.orientation.y
        self.copter_pose_sa.orientation.z = msg.pose.pose.orientation.z
        self.copter_pose_sa.orientation.w = msg.pose.pose.orientation.w
        
        (r,p,y) = self.convert_quat_to_euler(self.copter_pose_sa.orientation)
        self.copter_roll_sa = r
        self.copter_pitch_sa = p
        self.copter_yaw_sa = y

        self.copter_twist_sa.linear.x = msg.twist.twist.linear.vector.x
        self.copter_twist_sa.linear.y = msg.twist.twist.linear.vector.y
        self.copter_twist_sa.linear.z = msg.twist.twist.linear.vector.z
        self.copter_twist_sa.angular.x = msg.twist.twist.angular.vector.x
        self.copter_twist_sa.angular.y = msg.twist.twist.angular.vector.y
        self.copter_twist_sa.angular.z = msg.twist.twist.angular.vector.z
        return

    def read_blimp_state_sa(self,msg):
        self.blimp_pose_sa.position.x = msg.pose.pose.position.x
        self.blimp_pose_sa.position.y = msg.pose.pose.position.y
        self.blimp_pose_sa.position.z = msg.pose.pose.position.z
        self.blimp_pose_sa.orientation.x = msg.pose.pose.orientation.x
        self.blimp_pose_sa.orientation.y = msg.pose.pose.orientation.y
        self.blimp_pose_sa.orientation.z = msg.pose.pose.orientation.z
        self.blimp_pose_sa.orientation.w = msg.pose.pose.orientation.w

        (r,p,y) = self.convert_quat_to_euler(self.blimp_pose_sa.orientation)
        self.blimp_roll_sa = r
        self.blimp_pitch_sa = p
        self.blimp_yaw_sa = y

        self.blimp_twist_sa.linear.x = msg.twist.twist.linear.vector.x
        self.blimp_twist_sa.linear.y = msg.twist.twist.linear.vector.y
        self.blimp_twist_sa.linear.z = msg.twist.twist.linear.vector.z
        self.blimp_twist_sa.angular.x = msg.twist.twist.angular.vector.x
        self.blimp_twist_sa.angular.y = msg.twist.twist.angular.vector.y
        self.blimp_twist_sa.angular.z = msg.twist.twist.angular.vector.z
        return

    # def read_nn_coeffs_enu(self,msg):
    #     coeffs = list(msg.data)
    #     self.enu_ax = coeffs[0]
    #     self.enu_bx = coeffs[1]
    #     self.enu_cx = coeffs[2]
    #     self.enu_dx = coeffs[3]
    #     self.enu_ay = coeffs[4]
    #     self.enu_by = coeffs[5]
    #     self.enu_cy = coeffs[6]
    #     self.enu_dy = coeffs[7]
    #     self.enu_az = coeffs[8]
    #     self.enu_bz = coeffs[9]
    #     self.enu_cz = coeffs[10]
    #     self.enu_dz = coeffs[11]
    #     return   
    
    # def read_nn_coeffs_sa(self,msg):
    #     coeffs = list(msg.data)
    #     self.sa_ax = coeffs[0]
    #     self.sa_bx = coeffs[1]
    #     self.sa_cx = coeffs[2]
    #     self.sa_dx = coeffs[3]
    #     self.sa_ay = coeffs[4]
    #     self.sa_by = coeffs[5]
    #     self.sa_cy = coeffs[6]
    #     self.sa_dy = coeffs[7]
    #     self.sa_az = coeffs[8]
    #     self.sa_bz = coeffs[9]
    #     self.sa_cz = coeffs[10]
    #     self.sa_dz = coeffs[11]
    #     return 
    


    def episode_manager(self):
        if self.gust_status in [1,2,3]:
            self.log_response()
            self.reset_completed = False

        elif self.gust_status == 4 and not self.reset_completed:
            #Update progress bar
            if self.params["logging"]["show_progress_bar"]: print_progress_bar(self.i,self.params["logging"]["max_number_episodes"],prefix="Tot. "+str(self.params["logging"]["max_number_episodes"])+" episodes",suffix="complete",length=50)
            #Update episode counter and check terminal criterion
            self.i += 1
            if self.i >= self.params["logging"]["max_number_episodes"]: 
                self.node_shutdown_requested = True
            #Reset simulation
            self.init_models_in_sim()
            #Handle flags
            self.reset_completed = True

        else:
            #do nothing
            pass
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
            header_row += ["enu_blimp_px","enu_blimp_py","enu_blimp_pz"]
            header_row += ["enu_blimp_qx","enu_blimp_qy","enu_blimp_qz","enu_blimp_qw"]
            header_row += ["enu_blimp_roll","enu_blimp_pitch","enu_blimp_yaw"]
            header_row += ["enu_blimp_lin_vx","enu_blimp_lin_vy","enu_blimp_lin_vz"]
            header_row += ["enu_blimp_ang_vx","enu_blimp_ang_vy","enu_blimp_ang_vz"]
            
            header_row += ["enu_copter_px","enu_copter_py","enu_copter_pz"]
            header_row += ["enu_copter_qx","enu_copter_qy","enu_copter_qz","enu_copter_qw"]
            header_row += ["enu_copter_roll","enu_copter_pitch","enu_copter_yaw"]
            header_row += ["enu_copter_lin_vx","enu_copter_lin_vy","enu_copter_lin_vz"]
            header_row += ["enu_copter_ang_vx","enu_copter_ang_vy","enu_copter_ang_vz"]

            header_row += ["sa_blimp_px","sa_blimp_py","sa_blimp_pz"]
            header_row += ["sa_blimp_qx","sa_blimp_qy","sa_blimp_qz","blimp_qw"]
            header_row += ["sa_blimp_roll","sa_blimp_pitch","sa_blimp_yaw"]
            header_row += ["sa_blimp_lin_vx","sa_blimp_lin_vy","sa_blimp_lin_vz"]
            header_row += ["sa_blimp_ang_vx","sa_blimp_ang_vy","sa_blimp_ang_vz"]
            
            header_row += ["sa_copter_px","sa_copter_py","sa_copter_pz"]
            header_row += ["sa_copter_qx","sa_copter_qy","sa_copter_qz","copter_qw"]
            header_row += ["sa_copter_roll","sa_copter_pitch","sa_copter_yaw"]
            header_row += ["sa_copter_lin_vx","sa_copter_lin_vy","sa_copter_lin_vz"]
            header_row += ["sa_copter_ang_vx","sa_copter_ang_vy","sa_copter_ang_vz"]
            #Gust data
            header_row += ["pre_gust_duration"]
            header_row += ["gust_duration"]
            header_row += ["gust_break_duration"]
            header_row += ["episode_duration"]
            header_row += ["max_abs_gust_velocity"]
            header_row += ["norm_gust_dir_x","norm_gust_dir_y","norm_gust_dir_z"]            
            header_row += ["cur_gust_vel_x","cur_gust_vel_y","cur_gust_vel_z"]  
            header_row += ["gust_status"]
            # #nn coeffs
            # header_row += ["sa_ax","sa_bx","sa_cx","sa_dx"]
            # header_row += ["sa_ay","sa_by","sa_cy","sa_dy"]
            # header_row += ["sa_az","sa_bz","sa_cz","sa_dz"]
            # header_row += ["enu_ax","enu_bx","enu_cx","enu_dx"]
            # header_row += ["enu_ay","enu_by","enu_cy","enu_dy"]
            # header_row += ["enu_az","enu_bz","enu_cz","enu_dz"]            
            write_data_to_csv(log_file_path,header_row,self.log_verbose)
        else:
            #If log file already exists
            new_row = []
            #General data
            new_row += [self.header.stamp.to_sec()]
            #Blimp data
            new_row += [self.blimp_pose_gazebo.position.x,self.blimp_pose_gazebo.position.y,self.blimp_pose_gazebo.position.z]
            new_row += [self.blimp_pose_gazebo.orientation.x,self.blimp_pose_gazebo.orientation.y,self.blimp_pose_gazebo.orientation.z,self.blimp_pose_gazebo.orientation.w]
            new_row += [self.blimp_roll_gazebo,self.blimp_pitch_gazebo,self.blimp_yaw_gazebo]
            new_row += [self.blimp_twist_gazebo.linear.x,self.blimp_twist_gazebo.linear.y,self.blimp_twist_gazebo.linear.z]
            new_row += [self.blimp_twist_gazebo.angular.x,self.blimp_twist_gazebo.angular.y,self.blimp_twist_gazebo.angular.z]
            
            new_row += [self.copter_pose_gazebo.position.x,self.copter_pose_gazebo.position.y,self.copter_pose_gazebo.position.z]
            new_row += [self.copter_pose_gazebo.orientation.x,self.copter_pose_gazebo.orientation.y,self.copter_pose_gazebo.orientation.z,self.copter_pose_gazebo.orientation.w]
            new_row += [self.copter_roll_gazebo,self.copter_pitch_gazebo,self.copter_yaw_gazebo]
            new_row += [self.copter_twist_gazebo.linear.x,self.copter_twist_gazebo.linear.y,self.copter_twist_gazebo.linear.z]
            new_row += [self.copter_twist_gazebo.angular.x,self.copter_twist_gazebo.angular.y,self.copter_twist_gazebo.angular.z]

            new_row += [self.blimp_pose_sa.position.x,self.blimp_pose_sa.position.y,self.blimp_pose_sa.position.z]
            new_row += [self.blimp_pose_sa.orientation.x,self.blimp_pose_sa.orientation.y,self.blimp_pose_sa.orientation.z,self.blimp_pose_sa.orientation.w]
            new_row += [self.blimp_roll_sa,self.blimp_pitch_sa,self.blimp_yaw_sa]
            new_row += [self.blimp_twist_sa.linear.x,self.blimp_twist_sa.linear.y,self.blimp_twist_sa.linear.z]
            new_row += [self.blimp_twist_sa.angular.x,self.blimp_twist_sa.angular.y,self.blimp_twist_sa.angular.z]
            
            
            new_row += [self.copter_pose_sa.position.x,self.copter_pose_sa.position.y,self.copter_pose_sa.position.z]
            new_row += [self.copter_pose_sa.orientation.x,self.copter_pose_sa.orientation.y,self.copter_pose_sa.orientation.z,self.copter_pose_sa.orientation.w]
            new_row += [self.copter_roll_sa,self.copter_pitch_sa,self.copter_yaw_sa]
            new_row += [self.copter_twist_sa.linear.x,self.copter_twist_sa.linear.y,self.copter_twist_sa.linear.z]
            new_row += [self.copter_twist_sa.angular.x,self.copter_twist_sa.angular.y,self.copter_twist_sa.angular.z]
                        
            #Gust data
            new_row += [self.pre_gust_duration]
            new_row += [self.gust_duration]
            new_row += [self.gust_break_duration]
            new_row += [self.episode_duration]
            new_row += [self.max_abs_gust_velocity]
            new_row += [self.normalized_gust_direction.x,self.normalized_gust_direction.y,self.normalized_gust_direction.z]
            new_row += [self.cur_gust_velocity.x,self.cur_gust_velocity.y,self.cur_gust_velocity.z]
            new_row += [self.gust_status]
            # # nn coeffs
            # new_row += [self.sa_ax  ,self.sa_bx  ,self.sa_cx  ,self.sa_dx]
            # new_row += [self.sa_ay  ,self.sa_by  ,self.sa_cy  ,self.sa_dy]
            # new_row += [self.sa_az  ,self.sa_bz  ,self.sa_cz  ,self.sa_dz]
            # new_row += [self.enu_ax ,self.enu_bx ,self.enu_cx ,self.enu_dx]
            # new_row += [self.enu_ay ,self.enu_by ,self.enu_cy ,self.enu_dy]
            # new_row += [self.enu_az ,self.enu_bz ,self.enu_cz ,self.enu_dz]  

            dtype_new_row = self.convert_float_type(new_row,self.params["logging"]["cast_float_type"])
            write_data_to_csv(log_file_path,dtype_new_row,self.log_verbose)
        return
    
    def read_wind_gust_information(self,msg):
        self.abs_gust_velocity = msg.data[0]
        self.cur_gust_velocity.x = msg.data[1]
        self.cur_gust_velocity.y = msg.data[2]
        self.cur_gust_velocity.z = msg.data[3]
        self.max_abs_gust_velocity = msg.data[4]
        self.pre_gust_duration = msg.data[5]
        self.gust_duration = msg.data[6]
        self.gust_break_duration = msg.data[7]
        self.normalized_gust_direction.x = msg.data[8]
        self.normalized_gust_direction.y = msg.data[9]
        self.normalized_gust_direction.z = msg.data[10]
        self.gust_status = int(msg.data[11])
        self.header.stamp = rospy.Time.now()
        return

    def init_models_in_sim(self):
        #Init blimp
        blimp_init = ModelState()
        blimp_init.model_name = "blimp"
        blimp_init.pose.position.x = self.blimp_init_px
        blimp_init.pose.position.y = self.blimp_init_py
        blimp_init.pose.position.z = self.blimp_init_pz
        #Order of rotation: roll first, pitch second, yaw third
        q = quaternion_from_euler(self.blimp_init_roll,self.blimp_init_pitch,self.blimp_init_yaw)
        blimp_init.pose.orientation.x = q[0]
        blimp_init.pose.orientation.y = q[1]
        blimp_init.pose.orientation.z = q[2]
        blimp_init.pose.orientation.w = q[3]
        blimp_init.twist.linear.x = self.blimp_init_vx
        blimp_init.twist.linear.y = self.blimp_init_vy
        blimp_init.twist.linear.z = self.blimp_init_vz
        blimp_init.twist.angular.x = 0
        blimp_init.twist.angular.y = 0
        blimp_init.twist.angular.z = 0
        self.set_model_state_service(blimp_init)

        #Init copter
        copter_init = ModelState()
        copter_init.model_name = self.drone_name
        copter_init.pose.position.x = self.copter_init_px
        copter_init.pose.position.y = self.copter_init_py
        copter_init.pose.position.z = self.copter_init_pz
        #Order of rotation: roll first, pitch second, yaw third
        q = quaternion_from_euler(self.copter_init_roll,self.copter_init_pitch,self.copter_init_yaw)
        copter_init.pose.orientation.x = q[0]
        copter_init.pose.orientation.y = q[1]
        copter_init.pose.orientation.z = q[2]
        copter_init.pose.orientation.w = q[3]
        copter_init.twist.linear.x = self.copter_init_vx
        copter_init.twist.linear.y = self.copter_init_vy
        copter_init.twist.linear.z = self.copter_init_vz
        copter_init.twist.angular.x = 0
        copter_init.twist.angular.y = 0
        copter_init.twist.angular.z = 0
        self.set_model_state_service(copter_init)
        return

    def read_uav_state(self,msg):
        
        self.copter_pose_gazebo.position.x = msg.pose.pose.position.x
        self.copter_pose_gazebo.position.y = msg.pose.pose.position.y
        self.copter_pose_gazebo.position.z = msg.pose.pose.position.z
        self.copter_pose_gazebo.orientation.x = msg.pose.pose.orientation.x
        self.copter_pose_gazebo.orientation.y = msg.pose.pose.orientation.y
        self.copter_pose_gazebo.orientation.z = msg.pose.pose.orientation.z
        self.copter_pose_gazebo.orientation.w = msg.pose.pose.orientation.w
        (self.copter_roll_gazebo,self.copter_pitch_gazebo,self.copter_yaw_gazebo) = self.convert_quat_to_euler(self.copter_pose_gazebo.orientation)
        self.copter_twist_gazebo.linear.x = msg.twist.twist.linear.vector.x
        self.copter_twist_gazebo.linear.y = msg.twist.twist.linear.vector.y
        self.copter_twist_gazebo.linear.z = msg.twist.twist.linear.vector.z
        return
    
    def read_mp_state(self,msg):
         #Store data
        self.header.stamp = rospy.Time.now()
        self.blimp_pose_gazebo.position.x = msg.pose.pose.position.x
        self.blimp_pose_gazebo.position.y = msg.pose.pose.position.y
        self.blimp_pose_gazebo.position.z = msg.pose.pose.position.z
        self.blimp_twist_gazebo.linear.x = msg.twist.twist.linear.vector.x
        self.blimp_twist_gazebo.linear.y = msg.twist.twist.linear.vector.y
        self.blimp_twist_gazebo.linear.z = msg.twist.twist.linear.vector.z
        self.blimp_pose_gazebo.orientation.x = msg.pose.pose.orientation.x
        self.blimp_pose_gazebo.orientation.y = msg.pose.pose.orientation.y
        self.blimp_pose_gazebo.orientation.z = msg.pose.pose.orientation.z
        self.blimp_pose_gazebo.orientation.w = msg.pose.pose.orientation.w
        (self.blimp_roll_gazebo,self.blimp_pitch_gazebo,self.blimp_yaw_gazebo) = self.convert_quat_to_euler(self.blimp_pose_gazebo.orientation)

        return

if __name__ == '__main__':

    gust_response_recorder = GustDockingRecorder()
   
    #Init rate
    rate = rospy.Rate(gust_response_recorder.publish_hz)
    print("publish_hz",gust_response_recorder.publish_hz)

    if gust_response_recorder.params["logging"]["show_progress_bar"]: print_progress_bar(0,gust_response_recorder.params["logging"]["max_number_episodes"],prefix="Tot. "+str(gust_response_recorder.params["logging"]["max_number_episodes"])+" episodes",suffix="complete",length=50)
    #commands to be executed as long as node is up
    while not rospy.is_shutdown():
        gust_response_recorder.episode_manager()
        if gust_response_recorder.node_shutdown_requested:
            rospy.signal_shutdown("Maximum number of episodes reached. Shutdown initiated...")
        rate.sleep()
