'''
Class publishes all frames relevant for docking using the anafi drone and the apriltag detection. The script makes sure that the apriltag detection is considering the latency in the camera transmission.
'''
import math
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_inverse, quaternion_multiply, quaternion_from_matrix,concatenate_matrices,translation_from_matrix,translation_matrix,quaternion_matrix,quaternion_from_matrix, inverse_matrix
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped
from geometry_msgs.msg import PoseStamped,Quaternion, PointStamped
from anafi_control.msg import State
from geometry_msgs.msg import Vector3Stamped, PoseWithCovarianceStamped
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
import time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from olympe_bridge.kalman_filter import KalmanPosVelWithGPSBias2D
from olympe_bridge.transformation_GPS import GPS2ECEF, ECEF2NED, GPS2NED


node_name = 'anafi_coordinate_frames_publisher'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','anafi')

#Odometry topic of the drone




class AnafiTfFramesPublisher():
    def __init__(self):
        self.init_subscribers()
        self.init_variables()
        self.init_tf()
        self.init_parameters()
        self.init_publishers()
        return
    
    def init_publishers(self):
        self.p_tag_w_pub = rospy.Publisher("/tag_detection/p_tag_w",Vector3Stamped,queue_size=1)
    
    def init_parameters(self):
        self.gimbal_offset_x = 0.1
        self.gimbal_offset_y = 0
        self.gimbal_offset_z = 0.014
    
    

    def init_variables(self):
        self.gimbal_absolute = Vector3Stamped() # roll, pitch, yaw in deg. roll and pitch relative to frame similar to stability frame, not body-fixed frame. yaw relative to world frame axis, i.e. it is same than drone yaw angle.
        self.drone_state = State()
        self.apriltags = AprilTagDetectionArray()
        self.apriltag_pose = PoseStamped()
        self.home_location = PointStamped()
        self.speed = Vector3Stamped()
        self.gps_location = NavSatFix()
        self.home_position = Vector3Stamped()
        self.gps_position = Vector3Stamped()
        self.altitude = 0
        self.kf = KalmanPosVelWithGPSBias2D(
            dt = 0.02,
            sigma_acc = 1.0,      # (m/s^2) process noise driving velocity (maneuvers)
            sigma_bias= 0.06,    # (m) per-step bias process noise scale (wander rate)
            tau_bias = 120.0,     # (s) bias correlation time (b is slow if tau is large)
            sigma_vel_meas = 0.2, # (m/s) velocity measurement std
            sigma_gps_meas = 4.0, # (m) GPS measurement std (white part)
            )

        return

    def init_subscribers(self):
        self.state_subscriber = rospy.Subscriber("/"+drone_name+"/position_control/state_nwu",State,self.read_state)    
        self.gimbal_subscriber = rospy.Subscriber("/"+drone_name+"/gimbal/absolute",Vector3Stamped,self.read_gimbal)    
        self.apriltag_subscriber = rospy.Subscriber("/tag_detections",AprilTagDetectionArray,self.read_apriltag)
        self.speed_subscriber = rospy.Subscriber("/"+drone_name+"/drone/speed",Vector3Stamped,self.read_speed)
        self.gps_location_subscriber = rospy.Subscriber("/"+drone_name+"/drone/location_slow",NavSatFix,self.read_gps_location)
        self.home_location_subscriber = rospy.Subscriber("/"+drone_name+"/drone/location_slow",NavSatFix,self.read_home_location)
        self.altitude_subscriber = rospy.Subscriber("/"+drone_name+"/drone/altitude",Float32,self.read_altitude)
        return
    
        
    
    def init_tf(self):
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer)
        self.br = TransformBroadcaster()

    def publish_tfs(self):
        self.publish_stability_frame()
        self.publish_body_fixed_frame()
        self.publish_gimbal_frame()
        self.publish_kf_drone_frame()
        return

    def publish_stability_frame(self):
        """Function publishes the stability axes frame w.r.t. the world frame."""
        # Coordinate frame of anafi drone: 

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = drone_name + "/stability_axes"
        t.transform.translation.x = self.drone_state.pose.pose.position.x
        t.transform.translation.y = self.drone_state.pose.pose.position.y
        t.transform.translation.z = self.drone_state.pose.pose.position.z
        
        phi,theta,psi = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])
        q = quaternion_from_euler(0, 0, psi)
        
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
        
        # q = quaternion_from_euler(phi,theta,psi)

        t.transform.rotation.x = self.drone_state.pose.pose.orientation.x
        t.transform.rotation.y = self.drone_state.pose.pose.orientation.y
        t.transform.rotation.z = self.drone_state.pose.pose.orientation.z
        t.transform.rotation.w = self.drone_state.pose.pose.orientation.w   
        self.br.sendTransform(t)
        # print(t.header.stamp,"Anafi control: Stability axes frame transformation sent.",drone_name + "/stability_axes")
        return

    def publish_gimbal_frame(self,):
        """Function publishes the stability axes frame w.r.t. the world frame."""
        
        
        # p_drone = np.array([self.drone_state.pose.pose.position.x, self.drone_state.pose.pose.position.y, self.drone_state.pose.pose.position.z])
        p_drone = self.p_kf_drone_w
        q_drone = np.array([self.drone_state.pose.pose.orientation.x, self.drone_state.pose.pose.orientation.y, self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])
        
        # print('p_drone =',p_drone)
        
        #Get euler angles of body-fixed frame of anafi drone with regard to the world frame. 
        #The orientation of the anafi drone follows a north-west-up coordinate system

        phi,theta,psi = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])

        #The gimbal position changes in the world frame due to the offset to the center of the anafi drone when the drone changes its attitude.
        #Some signs are negative to account for the coordinate frame that is used by the parrot anafi drone (north-west-up --> north-east-down?)
        cr = np.cos(phi)
        sr = np.sin(phi)
        cp = np.cos(theta) #-
        sp = np.sin(theta)#-
        cy = np.cos(psi)#-
        sy = np.sin(psi)#-

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
        # print('p_gimbal_w =',p_gimbal_w)

        # print("p_gimbal_w =",p_gimbal_w)

        #Build the transformation
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = drone_name + "/gimbal_frame"
        t.transform.translation.x = p_gimbal_w[0]
        t.transform.translation.y = p_gimbal_w[1]
        t.transform.translation.z = p_gimbal_w[2]

        #Calculate gimbal orientation considering compensation capabilities of anafi drone (the drones roll and pitch angle is compensated, then the commanded roll and pitch angle are added)
        gimbal_roll = phi +   np.deg2rad(self.gimbal_absolute.vector.x) # compensation of drone roll + commanded gimbal roll angle
        gimbal_pitch = theta  + np.deg2rad(self.gimbal_absolute.vector.y) # compensation of drone pitch + commanded gimbal pitch angle
        gimbal_yaw = 0  # yaw of gimbal is fixed

        #Build the rotation of the quaternion
        q = quaternion_from_euler(-phi + gimbal_roll, -theta + gimbal_pitch, psi + gimbal_yaw) # correct pitch angle sign to make sure that rviz displays tf frames correctly.
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]    
        self.br.sendTransform(t)

        t_c = TransformStamped()
        t_c.header.stamp = rospy.Time.now()
        t_c.header.frame_id = drone_name + "/gimbal_frame"
        t_c.child_frame_id =  drone_name + "/camera_frame"
        t_c.transform.translation.x = 0
        t_c.transform.translation.y = 0
        t_c.transform.translation.z = 0
        q_c = quaternion_from_euler(0,-math.pi/2.0,-math.pi/2.0) 
        t_c.transform.rotation.x = q_c[0]
        t_c.transform.rotation.y = q_c[1]
        t_c.transform.rotation.z = q_c[2]
        t_c.transform.rotation.w = q_c[3]
        self.br.sendTransform(t_c)

        T_bfw = concatenate_matrices(translation_matrix(p_drone),quaternion_matrix(q_drone))
        T_wbf = inverse_matrix(T_bfw)

        t_bf = translation_from_matrix(T_wbf)
        q_bf = quaternion_from_matrix(T_wbf)
        
        t_sp = TransformStamped()
        t_sp.header.stamp = rospy.Time.now()
        t_sp.header.frame_id = drone_name + "/body_fixed"
        t_sp.child_frame_id =  drone_name + "/starting_point"
        t_sp.transform.translation.x = t_bf[0]
        t_sp.transform.translation.y = t_bf[1]
        t_sp.transform.translation.z = t_bf[2]
        t_sp.transform.rotation.x = q_bf[0]
        t_sp.transform.rotation.y = q_bf[1]
        t_sp.transform.rotation.z = q_bf[2]
        t_sp.transform.rotation.w = q_bf[3]
        self.br.sendTransform(t_sp)
        return
    
    def publish_apriltag_tf(self):
        p_drone_w = np.array([self.drone_state.pose.pose.position.x, self.drone_state.pose.pose.position.y, self.drone_state.pose.pose.position.z])
        #q_drone = np.array([self.drone_state.pose.pose.orientation.x, self.drone_state.pose.pose.orientation.y, self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])

         #Get euler angles of body-fixed frame of anafi drone with regard to the world frame
        phi,theta,psi = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])

        #The gimbal position changes in the world frame due to the offset to the center of the anafi drone when the drone changes its attitude.
        #Some signs are negative to account for the coordinate frame that is used by the parrot anafi drone (north-west-up --> north-east-down?)
        cr = np.cos(phi)
        sr = np.sin(phi)
        cp = np.cos(theta)
        sp = np.sin(theta)
        cy = np.cos(psi)
        sy = np.sin(psi)

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

        #Determining camera coordinate frame
        #Own experiments: when the tag is moved the following can be observed in the camera frame following applies:
        #x: positive when tag is moved to the right in the camera image --> x-axis to the right
        #y: positive when tag is moved down in the camera image --> y-axis down
        #z: positive when tag is moved away from the camera --> z-axis forward, out of the camera

        #The gimbal frame orientation is:
        #x: forward
        #y: left when looking from origin towards the tip of the x-axis
        #z: upward


        #Transformation from camera to gimbal:
        # x_g = z_c
        # y_g = -x_c
        # z_g = -y_c


        #Leads to transformation matrix:
        # R_gc = np.array([
        #    [0,0,1],
        #    [-1,0,0],
        #    [0,-1,0]
        #])

        #Calculate gimbal orientation considering compensation capabilities of anafi drone (the drones roll and pitch angle is compensated, then the commanded roll and pitch angle are added)
        cr_g = np.cos(np.deg2rad(self.gimbal_absolute.vector.x))
        sr_g = np.sin(np.deg2rad(self.gimbal_absolute.vector.x))
        cp_g = np.cos(np.deg2rad(self.gimbal_absolute.vector.y))
        sp_g = np.sin(np.deg2rad(self.gimbal_absolute.vector.y))
        cy_g = np.cos(psi)
        sy_g = np.sin(psi)

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
            [-1,0,0],
            [0,-1,0]

        ])
        p_tag_g = R_gc @ p_tag_c

        p_tag_w = Rwg @ (p_tag_g )+ p_gimbal_w

        msg_p_tag_w = Vector3Stamped()
        msg_p_tag_w.header.frame_id = "world"
        msg_p_tag_w.header.stamp = rospy.Time.now()
        
        msg_p_tag_w.vector.x = p_tag_w[0]
        msg_p_tag_w.vector.y = p_tag_w[1]
        msg_p_tag_w.vector.z = p_tag_w[2]
        self.p_tag_w_pub.publish(msg_p_tag_w)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id =  "detected_tag"
        t.transform.translation.x = p_tag_w[0]
        t.transform.translation.y = p_tag_w[1]
        t.transform.translation.z = p_tag_w[2]


        #Ignore tag orientation for now, just use default quaternion
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        self.br.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = drone_name +'/gimbal_frame'
        t.child_frame_id =  "detected_tag_gimbal"
        t.transform.translation.x = p_tag_g[0]
        t.transform.translation.y = p_tag_g[1]
        t.transform.translation.z = p_tag_g[2]


        #Ignore tag orientation for now, just use default quaternion
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        self.br.sendTransform(t)

        return

    def publish_kf_drone_frame(self):
        kf_position_x, kf_position_y,kf_position_z = self.kf.position[0],self.kf.position[1],self.altitude

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id =  drone_name +'/kf_body_frame'
        t.transform.translation.x = kf_position_x
        t.transform.translation.y = kf_position_y
        t.transform.translation.z = kf_position_z
        self.p_kf_drone_w = np.array([kf_position_x,kf_position_y,kf_position_z])


        #Ignore tag orientation for now, just use default quaternion
        hi,theta,psi = euler_from_quaternion([self.drone_state.pose.pose.orientation.x,self.drone_state.pose.pose.orientation.y,self.drone_state.pose.pose.orientation.z,self.drone_state.pose.pose.orientation.w])
        q = quaternion_from_euler(0, 0, psi)
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]    
        self.br.sendTransform(t)

    
    def rpyvec_nwu_to_enu(self,vec_nwu):
        """
        vec_nwu.vector.x = roll  (deg)
        vec_nwu.vector.y = pitch (deg)
        vec_nwu.vector.z = yaw   (deg)
        Angles are in NWU world frame. Convert to ENU world frame.
        """
        out = Vector3Stamped()
        out.header = vec_nwu.header  # keep stamp/frame_id

        # --- degrees -> radians ---
        r = math.radians(vec_nwu.vector.x)
        p = math.radians(vec_nwu.vector.y)
        y = math.radians(vec_nwu.vector.z)

        # --- NWU RPY -> quaternion (tf uses intrinsic RPY about fixed axes x,y,z) ---
        q_nwu = quaternion_from_euler(r, p, y)

        # --- frame rotation NWU -> ENU is +90 deg about Z ---
        q_rot = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)

        # --- apply: q_enu = q_rot ⊗ q_nwu ---
        q_enu = quaternion_multiply(q_rot, q_nwu)

        # normalize (good practice)
        n = math.sqrt(sum(v*v for v in q_enu))
        q_enu = [v / n for v in q_enu]

        # --- quaternion -> ENU RPY ---
        r_enu, p_enu, y_enu = euler_from_quaternion(q_enu)

        # --- radians -> degrees ---
        out.vector.x = math.degrees(r_enu)
        out.vector.y = math.degrees(p_enu)
        out.vector.z = math.degrees(y_enu)

        return out

    def vector3stamped_nwu_to_enu(self,v_nwu):
        """
        Convert a Vector3Stamped from NWU (x=N, y=W, z=U) to ENU (x=E, y=N, z=U).

        This is for true vectors (position delta, velocity, acceleration, force, etc.),
        NOT for Euler angles packed into x/y/z.
        """
        v_enu = Vector3Stamped()
        v_enu.header = v_nwu.header  # keep timestamp; you may want to change frame_id

        v_enu.vector.x = -v_nwu.vector.y  # East  = -West
        v_enu.vector.y =  v_nwu.vector.x  # North =  North
        v_enu.vector.z =  v_nwu.vector.z  # Up    =  Up

        # Optional: if you're truly converting frames, update the frame_id accordingly:
        # v_enu.header.frame_id = "world"  # or "map", etc.

        return v_enu


    def pose_nwu_to_enu(self,pose_nwu):
        pose_enu = PoseStamped()

        # ---- Position (NWU -> ENU) ----
        # NWU: x=N, y=W, z=U
        # ENU: x=E, y=N, z=U
        pose_enu.header = pose_nwu.header
        pose_enu.pose.position.x = -pose_nwu.pose.position.y   # E = -W
        pose_enu.pose.position.y =  pose_nwu.pose.position.x   # N =  N
        pose_enu.pose.position.z =  pose_nwu.pose.position.z   # U =  U

        # ---- Orientation (NWU -> ENU) ----
        # q_rot = +90deg about Z and =180deg about x. 
        # The rotation about x is necessary to account for the angle sign convention commonly used for UAVs.
        q_rot = quaternion_from_euler(math.pi, 0.0, math.pi / 2.0)

        q_nwu = [
            pose_nwu.pose.orientation.x,
            pose_nwu.pose.orientation.y,
            pose_nwu.pose.orientation.z,
            pose_nwu.pose.orientation.w,
        ]

        # Quaternion multiply: q_enu = q_rot * q_nwu
        q_enu = quaternion_multiply(q_rot, q_nwu)

        # Normalize (good practice)
        norm = math.sqrt(sum(v*v for v in q_enu))
        q_enu = [v / norm for v in q_enu]

        pose_enu.pose.orientation.x = q_enu[0]
        pose_enu.pose.orientation.y = q_enu[1]
        pose_enu.pose.orientation.z = q_enu[2]
        pose_enu.pose.orientation.w = q_enu[3]
        # print("Converted pose",pose_enu)
        return pose_enu
    
    def read_state(self,msg): 
        """Function reads the Sphinx message from the sphinx interface node.
        """
        self.drone_state.pose = self.pose_nwu_to_enu(msg.pose)
        self.drone_state.twist.twist.linear = self.vector3stamped_nwu_to_enu(msg.twist.twist.linear)
        self.drone_state.twist.twist.angular = self.vector3stamped_nwu_to_enu(msg.twist.twist.angular)
        # self.drone_state.pose.pose.position.z = -self.drone_state.pose.pose.position.z
        # self.publish_tfs()
        self.publish_stability_frame()
        self.publish_body_fixed_frame()
        return
    
    def read_gimbal(self,msg):
        self.gimbal_absolute = self.rpyvec_nwu_to_enu(msg)  
        self.publish_gimbal_frame()
        #self.publish_tfs()

    def read_apriltag(self,msg):
        self.apriltags = msg.detections
        if self.apriltags:
            apriltag_detection = self.apriltags[0] #for testing assume just one tag
            self.apriltag_pose = apriltag_detection.pose
            self.publish_apriltag_tf()
    
    def read_speed(self,msg):
        self.speed = self.vector3stamped_nwu_to_enu(msg)
        self.kf.update_velocity(self.speed.vector.x, self.speed.vector.y)

        return
    
    def read_gps_location(self,msg):
        self.gps_location = msg
        location_nwu = GPS2NED(msg.latitude, msg.longitude, msg.altitude, self.origin_ECEF,self.R_ECEF2NED)
        location_nwu[2] = self.altitude


        #Notice the order of points in the array
        gps_position_nwu = Vector3Stamped()
        gps_position_nwu.header.stamp = msg.header.stamp
        gps_position_nwu.vector.x,gps_position_nwu.vector.y,gps_position_nwu.vector.z = location_nwu[0],location_nwu[1],location_nwu[2]
        self.gps_position = self.vector3stamped_nwu_to_enu(gps_position_nwu)
        self.kf.update_gps(self.gps_position.vector.x, self.gps_position.vector.y)

        return
    
    def read_home_location(self,msg):
        self.home_location = msg
        if not math.isnan(msg.latitude) and self.home_location.latitude != 500.0:
            self.origin_ECEF,self.R_ECEF2NED = GPS2ECEF(self.home_location.latitude,self.home_location.longitude,0,1)
            self.home_position.header.stamp = rospy.Time.now()
            self.home_position.vector.x,self.home_position.vector.y,self.home_position.vector.z = self.origin_ECEF[0],self.origin_ECEF[1],self.origin_ECEF[2]
        return
    
    def read_altitude(self,msg):
        self.altitude = msg.data
        return
    
    



if __name__ == '__main__':
    #Initialize node
    rospy.init_node(node_name)
    anafi_tf_publisher = AnafiTfFramesPublisher()
    rospy.loginfo("publisher node for stability axes frame started ")
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        anafi_tf_publisher.kf.predict()
        print(anafi_tf_publisher.kf.position)
        print(anafi_tf_publisher.kf.velocity)
        print(anafi_tf_publisher.kf.gps_bias)
        print("--------------")
        anafi_tf_publisher.publish_kf_drone_frame()
        rate.sleep()