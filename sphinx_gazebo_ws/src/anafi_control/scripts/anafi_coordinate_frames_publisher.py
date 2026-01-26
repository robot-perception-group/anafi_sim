'''
Class publishes all frames relevant for docking using the anafi drone and the apriltag detection. The script makes sure that the apriltag detection is considering the latency in the camera transmission.
'''

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_inverse, quaternion_multiply, quaternion_from_matrix
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped
from geometry_msgs.msg import PoseStamped,Quaternion
from anafi_control.msg import State
from geometry_msgs.msg import Vector3Stamped, PoseWithCovarianceStamped
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection

node_name = 'anafi_control_publish_stability_axes_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','anafi')

#Odometry topic of the drone
state_topic = ("/"+drone_name+"/position_control/state_enu",State)
gimbal_topic = ("/"+drone_name+"/gimbal/absolute",Vector3Stamped)


class AnafiTfFramesPublisher():
    def __init__(self):
        self.init_subscribers()
        self.init_variables()
        self.init_tf()
        self.init_parameters()
        return
    
    def init_parameters(self):
        self.gimbal_offset_x = 0.1
        self.gimbal_offset_y = 0
        self.gimbal_offset_z = 0.014
    
    

    def init_variables(self):
        self.gimbal_absolute = Vector3Stamped() # roll, pitch, yaw in deg. roll and pitch relative to frame similar to stability frame, not body-fixed frame. yaw relative to world frame axis, i.e. it is same than drone yaw angle.
        self.drone_state = State()
        self.apriltags = AprilTagDetectionArray()
        self.apriltag_pose = PoseStamped()
        return

    def init_subscribers(self):
        self.state_subscriber = rospy.Subscriber("/"+drone_name+"/position_control/state_enu",State,self.read_state)    
        self.gimbal_subscriber = rospy.Subscriber("/"+drone_name+"/gimbal/absolute",Vector3Stamped,self.read_gimbal)    
        self.apriltag_subscriber = rospy.Subscriber("/tag_detections",AprilTagDetectionArray,self.read_apriltag)
        return
    
    def init_tf(self):
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer)
        self.br = TransformBroadcaster()

    def publish_tfs(self):
        self.publish_stability_frame()
        self.publish_body_fixed_frame()
        self.publish_gimbal_frame()
        return

    def publish_stability_frame(self):
        """Function publishes the stability axes frame w.r.t. the world frame."""

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = drone_name + "/stability_axes"
        t.transform.translation.x = self.drone_state.pose.pose.position.x
        t.transform.translation.y = self.drone_state.pose.pose.position.y
        t.transform.translation.z = self.drone_state.pose.pose.position.z
        
        phi,theta,psi = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])
        q = quaternion_from_euler(0, 0, -psi)
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]    
        self.br.sendTransform(t)
        # print(t.header.stamp,"Anafi control: Stability axes frame transformation sent.",drone_name + "/stability_axes")
        return

    def publish_body_fixed_frame(self):
        """Function publishes the stability axes frame w.r.t. the world frame."""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = drone_name + "/body_fixed"
        t.transform.translation.x = self.drone_state.pose.pose.position.x
        t.transform.translation.y = self.drone_state.pose.pose.position.y
        t.transform.translation.z = self.drone_state.pose.pose.position.z

        phi,theta,psi = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])
        
        q = quaternion_from_euler(phi,-theta,-psi)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]   
        self.br.sendTransform(t)
        # print(t.header.stamp,"Anafi control: Stability axes frame transformation sent.",drone_name + "/stability_axes")
        return

    def publish_gimbal_frame(self,):
        """Function publishes the stability axes frame w.r.t. the world frame."""
        
        
        p_drone = np.array([self.drone_state.pose.pose.position.x, self.drone_state.pose.pose.position.y, self.drone_state.pose.pose.position.z])
        
        
        #Get euler angles of body-fixed frame of anafi drone with regard to the world frame
        phi,theta,psi = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])

        #The gimbal position changes in the world frame due to the offset to the center of the anafi drone when the drone changes its attitude.
        #Some signs are negative to account for the coordinate frame that is used by the parrot anafi drone (north-west-up --> north-east-down?)
        cr = np.cos(phi)
        sr = np.sin(phi)
        cp = np.cos(-theta)
        sp = np.sin(-theta)
        cy = np.cos(-psi)
        sy = np.sin(-psi)

        #Calculate rotation matrix from body fixed to world frame
        R = np.array([
            [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
            [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
            [-sp,    cp*sr,             cp*cr           ]
        ])

        #Build the offset vector
        p_gimbal_bf = np.array([self.gimbal_offset_x,self.gimbal_offset_y,self.gimbal_offset_z])

        #Rotate the offset vector which is described in the 
        p_gimbal_w = R @ p_gimbal_bf + p_drone
    
        #Build the transformation
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = drone_name + "/gimbal_frame"
        t.transform.translation.x = p_gimbal_w[0]
        t.transform.translation.y = p_gimbal_w[1]
        t.transform.translation.z = p_gimbal_w[2]

        #Calculate gimbal orientation considering compensation capabilities of anafi drone (the drones roll and pitch angle is compensated, then the commanded roll and pitch angle are added)
        gimbal_roll = -phi +   np.deg2rad(self.gimbal_absolute.vector.x) # compensation of drone roll + commanded gimbal roll angle
        gimbal_pitch = theta  + np.deg2rad(self.gimbal_absolute.vector.y) # compensation of drone pitch + commanded gimbal pitch angle
        gimbal_yaw = 0  # yaw of gimbal is fixed

        #Build the rotation of the quaternion
        q = quaternion_from_euler(phi + gimbal_roll, -theta + gimbal_pitch, -psi + gimbal_yaw) # correct pitch angle sign to make sure that rviz displays tf frames correctly.
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]    
        self.br.sendTransform(t)
        return
    
    def publish_apriltag_tf(self):
        p_drone_w = np.array([self.drone_state.pose.pose.position.x, self.drone_state.pose.pose.position.y, self.drone_state.pose.pose.position.z])
        q_drone = np.array([self.drone_state.pose.pose.orientation.x, self.drone_state.pose.pose.orientation.y, self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])

         #Get euler angles of body-fixed frame of anafi drone with regard to the world frame
        phi,theta,psi = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])

        #The gimbal position changes in the world frame due to the offset to the center of the anafi drone when the drone changes its attitude.
        #Some signs are negative to account for the coordinate frame that is used by the parrot anafi drone (north-west-up --> north-east-down?)
        cr = np.cos(phi)
        sr = np.sin(phi)
        cp = np.cos(-theta)
        sp = np.sin(-theta)
        cy = np.cos(-psi)
        sy = np.sin(-psi)

        #Calculate rotation matrix from body fixed to world frame
        Rwbf = np.array([
            [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
            [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
            [-sp,    cp*sr,             cp*cr           ]
        ])

        #Build the offset vector
        p_gimbal_bf = np.array([self.gimbal_offset_x,self.gimbal_offset_y,self.gimbal_offset_z])

        #Rotate the offset vector which is described in the 
        p_gimbal_w = Rwbf @ p_gimbal_bf + p_drone_w

        #Own experiments: when the tag is moved the following can be observed in the camera frame following applies:
        #x: positive when tag is moved to the left in the camera image
        #y: positive when tag is moved down in the camera image
        #z: positive when tag is moved away from the camera
        #Therefore it follows:
        #x: forward, outwards the camera
        #y: left, when looking along the x-axis from origin towards the tip of the x-axis
        #z: upward.


        #Calculate gimbal orientation considering compensation capabilities of anafi drone (the drones roll and pitch angle is compensated, then the commanded roll and pitch angle are added)
        cr_g = np.cos(np.deg2rad(self.gimbal_absolute.vector.x))
        sr_g = np.sin(np.deg2rad(self.gimbal_absolute.vector.x))
        cp_g = np.cos(np.deg2rad(self.gimbal_absolute.vector.y))
        sp_g = np.sin(np.deg2rad(self.gimbal_absolute.vector.y))
        cy_g = np.cos(-psi)
        sy_g = np.sin(-psi)

        #Calculate rotation matrix from body fixed to world frame
        Rwg = np.array([
            [cy_g*cp_g,  cy_g*sp_g*sr_g - sy_g*cr_g,  cy_g*sp_g*cr_g + sy_g*sr_g],
            [sy_g*cp_g,  sy_g*sp_g*sr_g + cy_g*cr_g,  sy_g*sp_g*cr_g - cy_g*sr_g],
            [-sp_g,    cp_g*sr_g,             cp_g*cr_g           ]
        ])




    
        p_tag_c = np.array([self.apriltag_pose.pose.pose.position.x,self.apriltag_pose.pose.pose.position.y,self.apriltag_pose.pose.pose.position.z])
        #Create rotation matrix that rotates from camera to gimbal 
        R_gc = np.array([
            [0,0,1],
            [1,0,0],
            [0,-1,0]

        ])
        p_tag_g = R_gc @ p_tag_c

        #Use inverse in order to compensate for attitude changes of the anafi drone of the gimbal and thus the tag
        # p_tag_w = np.linalg.inv(Rwbf) @ (p_tag_g + p_gimbal_bf) +  p_drone
        p_tag_w = np.linalg.inv(Rwg) @ (p_tag_g )+ p_gimbal_w 

        #Handle rotation
        #CURRENTLY, TAG ORIENTATION IS NEGLECTED; ONLY POSITION IS OF IMPORTANCE
        # q_gc = quaternion_from_euler(np.pi/2,0,-np.pi/2)
        # q_tag_c =  np.array([self.apriltag_pose.pose.pose.orientation.x,self.apriltag_pose.pose.pose.orientation.y,self.apriltag_pose.pose.pose.orientation.z,self.apriltag_pose.pose.pose.orientation.w])
        
        # q_tag_g = quaternion_multiply(q_gc,q_tag_c)

        # q_g= quaternion_from_euler(np.deg2rad(self.gimbal_absolute.vector.x),np.deg2rad(self.gimbal_absolute.vector.y),-psi)

        # q_tag_w = quaternion_multiply(quaternion_inverse(q_g),q_tag_g)

        #deltas_tag_world = R @ deltas_tag
    
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id =  "detected_tag"
        t.transform.translation.x = p_tag_w[0]
        t.transform.translation.y = p_tag_w[1]
        t.transform.translation.z = p_tag_w[2]



        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        self.br.sendTransform(t)

        return


    
    def read_state(self,msg): 
        """Function reads the Sphinx message from the sphinx interface node.
        """
        self.drone_state = msg
        self.drone_state.pose.pose.position.z = 0
        self.publish_tfs()
        return
    
    def read_gimbal(self,msg):
        self.gimbal_absolute = msg  
        self.publish_tfs()

    def read_apriltag(self,msg):
        self.apriltags = msg.detections
        if self.apriltags:
            apriltag_detection = self.apriltags[0] #for testing assume just one tag
            self.apriltag_pose = apriltag_detection.pose
            self.publish_apriltag_tf()
    
    # def publish_apriltag_tf(self):
    #     t = TransformStamped()
    #     #Gimbal position relative to origin of the anafi drone
    #     #Determined offset values in sphinx

    #     phi,theta,psi = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])

    
    #     #Define rotation matrix to consider offset between center of drone and center of gimbal
    #     cr = np.cos(phi)
    #     sr = np.sin(phi)
    #     cp = np.cos(-theta)
    #     sp = np.sin(-theta)
    #     cy = np.cos(-psi)
    #     sy = np.sin(-psi)
    
    #     R = np.array([
    #         [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
    #         [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
    #         [-sp,    cp*sr,             cp*cr           ]
    #     ])
    
    #     deltas_bf = np.array([self.gimbal_offset_x,self.gimbal_offset_y,self.gimbal_offset_z])
    #     deltas_gimbal_world = R @ deltas_bf

    #     #Define rotation matrix to consider offset between center of gimbal and apriltag
    #     tcr = np.cos(phi)
    #     tsr = np.sin(phi)
    #     tcp = np.cos(-theta)
    #     tsp = np.sin(-theta)
    #     tcy = np.cos(-psi)
    #     tsy = np.sin(-psi)
    
        
    #     R = np.array([
    #         [tcy*tcp,  tcy*tsp*tsr - tsy*tcr,  tcy*tsp*tcr + tsy*tsr],
    #         [tsy*tcp,  tsy*tsp*tsr + tcy*tcr,  tsy*tsp*tcr - tcy*tsr],
    #         [-tsp,    tcp*tsr,             tcp*tcr           ]
    #     ])
    
    #     deltas_tag = np.array([self.apriltag_pose.pose.pose.position.z,-self.apriltag_pose.pose.pose.position.x,self.apriltag_pose.pose.pose.position.y])
    #     deltas_tag_world = R @ deltas_tag
    
    
    #     t.header.stamp = rospy.Time.now()
    #     t.header.frame_id = drone_name + '/gimbal_frame'
    #     t.child_frame_id =  "detected_tag"
    #     t.transform.translation.x = self.apriltag_pose.pose.pose.position.z
    #     t.transform.translation.y = self.apriltag_pose.pose.pose.position.x
    #     t.transform.translation.z = self.apriltag_pose.pose.pose.position.y

    #     #Calculate gimbal orientation considering compensation capabilities of anafi drone

    #     gimbal_roll = -phi +   np.deg2rad(self.gimbal_absolute.vector.x) # compensation of drone roll + commanded gimbal roll angle
    #     gimbal_pitch = theta  + np.deg2rad(self.gimbal_absolute.vector.y) # compensation of drone pitch + commanded gimbal pitch angle
    #     gimbal_yaw = 0  # yaw of gimbal is fixed


    #     t.transform.rotation.x = self.apriltag_pose.pose.pose.orientation.x
    #     t.transform.rotation.y = self.apriltag_pose.pose.pose.orientation.y
    #     t.transform.rotation.z = self.apriltag_pose.pose.pose.orientation.z
    #     t.transform.rotation.w = self.apriltag_pose.pose.pose.orientation.w    
    #     self.br.sendTransform(t)

    #     return




if __name__ == '__main__':
    #Initialize node
    rospy.init_node(node_name)
    anafi_tf_publisher = AnafiTfFramesPublisher()
    rospy.loginfo("publisher node for stability axes frame started ")
    rospy.spin()