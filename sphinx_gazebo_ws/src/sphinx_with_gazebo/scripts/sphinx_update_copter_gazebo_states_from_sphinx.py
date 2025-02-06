import rospy
from sphinx_with_gazebo.msg import Sphinx
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped, PoseStamped
from gazebo_msgs.srv import SetModelState, SetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from olympe_bridge.msg import GimbalCommand
import tf2_geometry_msgs
import tf2_ros

import time

# Parameters
node_name = "sphinx_update_copter_gazebo_states_from_sphinx_node"
publish_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/publish_hz',"50"))
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name',"anafi")


# Subscriber topics
anafi_data_topic = ("/anafi/sphinx/drone_data", Sphinx)
camera_attitude_topic = ("/anafi/camera/attitude",QuaternionStamped)


#Publisher topics
gimbal_control_roll = ("/"+drone_name+"/gimbal_roll_controller/command",Float64)
gimbal_control_pitch = ("/"+drone_name+"/gimbal_pitch_controller/command",Float64)

# Service topics
set_model_topic = ("/gazebo/set_model_state",SetModelState)



class ControlCopterFromSphinx():
    def __init__(self):
        #Variables
        self.sphinx_data = Sphinx()
        self.camera_attitude = QuaternionStamped()
        self.target_frame = "world"
        self.original_frame = drone_name+"/body_fixed"

        #Services
        rospy.wait_for_service(set_model_topic[0])
        self.set_model_state_service = rospy.ServiceProxy(set_model_topic[0],set_model_topic[1])

        #Subscribers
        self.sphinx_data_subscriber = rospy.Subscriber(anafi_data_topic[0],anafi_data_topic[1],self.read_sphinx_data)
        self.camera_base_subscriber = rospy.Subscriber(camera_attitude_topic[0],camera_attitude_topic[1],self.read_camera_attitude)

        #Publishers
        self.gimbal_pitch_controller_publisher = rospy.Publisher(gimbal_control_pitch[0],gimbal_control_pitch[1],queue_size=1)
        self.gimbal_roll_controller_publisher = rospy.Publisher(gimbal_control_roll[0],gimbal_control_roll[1],queue_size=1)

        #tf transformation
         #Init tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        return

    def read_sphinx_data(self,msg):
        if not np.isclose(msg.timeStamp,0):    
            self.sphinx_data = msg
        return
    
    def read_camera_attitude(self,msg):
        #Defined in body fixed frame
        
        q = [msg.quaternion.x,msg.quaternion.y,msg.quaternion.z,msg.quaternion.w]
        (roll, pitch, yaw) = np.rad2deg(euler_from_quaternion(q))
        
        print(f"cam attitude: {roll:02},{pitch:02},{yaw:02}")
        #switch roll and pitch and negate pitch because camera plugin is rotated by 90 deg around vertical axis
        q = quaternion_from_euler(np.deg2rad(roll),np.deg2rad(pitch),np.deg2rad(yaw + 90))
        self.camera_attitude.quaternion.x = q[0]
        self.camera_attitude.quaternion.y = q[1]
        self.camera_attitude.quaternion.z = q[2]
        self.camera_attitude.quaternion.w = q[3]
        # print("mod quat =",self.camera_attitude.quaternion)
        # print(roll,pitch)
        # self.gimbal_roll_controller_publisher.publish( Float64(roll))
        # self.gimbal_pitch_controller_publisher.publish(Float64(pitch))
        return
    
    def read_desired_roll_pitch(self,msg):
        self.desired_roll_pitch = msg
        return
    

    def set_model_state(self):
        new_state = ModelState()

        new_state.model_name = drone_name

        q = quaternion_from_euler(self.sphinx_data.attitudeX,self.sphinx_data.attitudeY,self.sphinx_data.attitudeZ)
        new_state.pose.orientation.x = q[0]
        new_state.pose.orientation.y = q[1]
        new_state.pose.orientation.z = q[2]
        new_state.pose.orientation.w = q[3]

        new_state.pose.position.x = self.sphinx_data.posX
        new_state.pose.position.y = self.sphinx_data.posY
        new_state.pose.position.z = self.sphinx_data.posZ

        self.set_model_state_service(new_state)

        return
    
    def set_camera_state(self):
        new_state = ModelState()

        new_state.model_name = drone_name+"_camera"

        cam_pose_world = self.get_cam_pose_in_world_frame()

       
        new_state.pose.orientation = cam_pose_world.pose.orientation


        new_state.pose.position = cam_pose_world.pose.position

        self.set_model_state_service(new_state)

        return
    

    def get_cam_pose_in_body_fixed_frame(self):
        #All values in body fixed frame
        cam_pose = PoseStamped()
        cam_pose.header.stamp = rospy.Time.now()
        cam_pose.pose.position.x = 0.1 # [m] offset of camera from base
        cam_pose.pose.position.y = 0 # [m] offset of camera from base
        cam_pose.pose.position.z = 0.005 # [m] offset of camera from base
        cam_pose.pose.orientation = self.camera_attitude.quaternion #camera attitude in body fixed frame given as quaternions
        return cam_pose
    
    def get_cam_pose_in_world_frame(self):
        #calculates cam pose in world frame
        cam_pose_bf = self.get_cam_pose_in_body_fixed_frame()


        
        
        cam_pose_world = PoseStamped()
        cam_pose_world.header.stamp = rospy.Time.now()

        try:
            trans_from_orig_frame_to_target_frame = self.tf_buffer.lookup_transform(self.target_frame,self.original_frame, rospy.Time())
            cam_pose_world = tf2_geometry_msgs.do_transform_pose(cam_pose_bf,trans_from_orig_frame_to_target_frame)
            cam_pose_world.header.frame_id = self.target_frame
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            warn_string = "Transformation from frame "+self.original_frame+" to " + self.target_frame +" threw an error."
            rospy.logwarn(warn_string)
            rate.sleep()


        #Add drone position
        # cam_pose_world.pose.position.x += self.sphinx_data.posX
        # cam_pose_world.pose.position.y += self.sphinx_data.posY
        # cam_pose_world.pose.position.z += self.sphinx_data.posZ
        return cam_pose_world
    

    # def publish_static_transform_for_anafi_camera(self):
    #     cam_pose_world = self.get_cam_pose_in_world_frame()
    #     t = tf2_ros.TransformStamped()
    #     t.header.stamp = rospy.Time.now()
    #     t.header.frame_id = self.original_frame
    #     t.child_frame_id = self.target_frame
    #     t.transform.translation.x = cam_pose_world.pose.position.x
    #     t.transform.translation.y = cam_pose_world.pose.position.y
    #     t.transform.translation.z = cam_pose_world.pose.position.z
    #     t.transform.rotation.x = cam_pose_world.pose.orientation.x
    #     t.transform.rotation.y = cam_pose_world.pose.orientation.y
    #     t.transform.rotation.z = cam_pose_world.pose.orientation.z
    #     t.transform.rotation.w = cam_pose_world.pose.orientation.w    
    #     self.transform_broadcaster.sendTransform(t)
    #     print(t.header.stamp,"Transform sent.")
    #     return




        


    
    


if __name__ == "__main__":
    rospy.init_node(node_name)

    setter = ControlCopterFromSphinx()
    
    print("publish_hz =",publish_hz)
    rate = rospy.Rate(publish_hz)

    while not rospy.is_shutdown():
        setter.set_model_state()
        setter.set_camera_state()
        # setter.publish_static_transform_for_anafi_camera()
        rate.sleep()




