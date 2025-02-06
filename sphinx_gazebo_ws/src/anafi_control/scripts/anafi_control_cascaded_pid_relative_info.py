import rospy
from std_msgs.msg import Float64, Float64MultiArray, Float32
import tf2_ros
from geometry_msgs.msg import TwistStamped, Vector3Stamped, PoseStamped,  Quaternion
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates, LinkStates
from gazebo_msgs.srv import GetModelState
import tf2_geometry_msgs 
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse, quaternion_from_euler
from anafi_control.msg import State, Waypoint
from sphinx_with_gazebo.msg import Sphinx
from anafi_control.msg import State
import numpy as np
from copy import deepcopy
from tf.transformations import quaternion_from_euler

#Parameters
node_name='cascaded_pid_relative_info_node'
relative_info_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/relative_info_hz',"10"))
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','anafi')


#Subscriber topics
waypoint_topic = ('/'+drone_name+'/position_control/waypoint',Waypoint)
sphinx_drone_topic = ('/'+drone_name+'/position_control/state_enu',State)



#Publisher topics
relative_vel_state_topic = ('drone_frame/relative_waypoint_drone/state/twist_ac',TwistStamped)
relative_pos_state_topic = ('drone_frame/relative_waypoint_drone/state/pose_ac',PoseStamped)
relative_vel_state_world_topic = ('wp_frame/relative_waypoint_drone/state/twist_ac',TwistStamped)
relative_pos_state_world_topic = ('wp_frame/relative_waypoint_drone/state/pose_ac',PoseStamped)
drone_state_topic = ('drone_frame/drone/state_ac',State)
target_state_topic = ('drone_frame/waypoint/state_ac',State)
drone_state_world_topic = ('world_frame/drone/state_ac',State)
target_state_world_topic = ('world_frame/waypoint/state_ac',State)
relative_rpy_topic = ("drone_frame/relative_waypoint_drone/roll_pitch_yaw_ac",Float64MultiArray)
drone_rpy_topic = ("drone_frame/drone/roll_pitch_yaw_ac",Float64MultiArray)
target_rpy_topic = ("drone_frame/waypoint/roll_pitch_yaw_ac",Float64MultiArray)

#Publisher definitions
relative_vel_state_publisher = rospy.Publisher(relative_vel_state_topic[0],relative_vel_state_topic[1],queue_size = 0)
relative_pos_state_publisher = rospy.Publisher(relative_pos_state_topic[0],relative_pos_state_topic[1],queue_size = 0)
relative_vel_state_world_publisher = rospy.Publisher(relative_vel_state_world_topic[0],relative_vel_state_world_topic[1],queue_size = 0)
relative_pos_state_world_publisher = rospy.Publisher(relative_pos_state_world_topic[0],relative_pos_state_world_topic[1],queue_size = 0)
relative_rpy_publisher = rospy.Publisher(relative_rpy_topic[0],relative_rpy_topic[1],queue_size=0)
drone_rpy_publisher = rospy.Publisher(drone_rpy_topic[0],drone_rpy_topic[1],queue_size=0)
target_rpy_publisher = rospy.Publisher(target_rpy_topic[0],target_rpy_topic[1],queue_size=0)

drone_state_publisher = rospy.Publisher(drone_state_topic[0],drone_state_topic[1],queue_size = 0)
target_state_publisher = rospy.Publisher(target_state_topic[0],target_state_topic[1],queue_size = 0)
drone_state_world_publisher = rospy.Publisher(drone_state_world_topic[0],drone_state_world_topic[1],queue_size = 0)
target_state_world_publisher = rospy.Publisher(target_state_world_topic[0],target_state_world_topic[1],queue_size = 0)

#Frame definitions
wp_frame = 'world'
drone_frame = drone_name+'/stability_axes'


#Class definition
class PoseTwistState():
    def __init__(self):
        """Function stores the pose and twist information together with the reference frame."""
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'world'

        self.twist = TwistStamped()
        self.twist.header.frame_id = 'world'
        self.twist.twist.linear = Vector3Stamped()
        self.twist.twist.angular = Vector3Stamped()
        return

#Class initialization
drone_state_in_wp_frame = PoseTwistState()
drone_state_in_wp_frame.pose.header.frame_id = wp_frame
drone_state_in_wp_frame.twist.header.frame_id = wp_frame
target_state_in_wp_frame = PoseTwistState()
target_state_in_wp_frame.pose.header.frame_id = wp_frame
target_state_in_wp_frame.twist.header.frame_id = wp_frame


drone_state_in_drone_frame = PoseTwistState()
target_state_in_drone_frame = PoseTwistState()



def read_drone(msg):

    drone_state_in_wp_frame.pose = msg.pose
    drone_state_in_wp_frame.twist = msg.twist



    return


#Write function for reading Waypoint here
def read_waypoint(msg):
    #TRansform data
    q = quaternion_from_euler(0,0,np.deg2rad(msg.yaw))

    #Store data
    target_state_in_wp_frame.pose.header.frame_id = wp_frame
    target_state_in_wp_frame.pose.pose.position.x = msg.x
    target_state_in_wp_frame.pose.pose.position.y = msg.y
    target_state_in_wp_frame.pose.pose.position.z = msg.z
    target_state_in_wp_frame.pose.pose.orientation.x = q[0]
    target_state_in_wp_frame.pose.pose.orientation.y = q[1]
    target_state_in_wp_frame.pose.pose.orientation.z = q[2]
    target_state_in_wp_frame.pose.pose.orientation.w = q[3]
    target_state_in_wp_frame.twist.header.frame_id = wp_frame
    target_state_in_wp_frame.twist.twist.linear.vector.x = msg.v_x
    target_state_in_wp_frame.twist.twist.linear.vector.y = msg.v_y
    target_state_in_wp_frame.twist.twist.linear.vector.z = msg.v_z
    return




def compute_relative_vel_msg(target_state,drone_state):
    '''
    Function computes the relative velocity vector between the moving platform and the drone.
    '''
    rel_vel_msg = TwistStamped()
    rel_vel_msg.header.stamp = rospy.Time.now()
    rel_vel_msg.header.frame_id = drone_frame
    rel_vel_msg.twist.linear.x = target_state.twist.twist.linear.vector.x-drone_state.twist.twist.linear.vector.x
    rel_vel_msg.twist.linear.y = target_state.twist.twist.linear.vector.y-drone_state.twist.twist.linear.vector.y
    rel_vel_msg.twist.linear.z = target_state.twist.twist.linear.vector.z-drone_state.twist.twist.linear.vector.z

    rel_vel_msg.twist.angular.x = target_state.twist.twist.angular.vector.x-drone_state.twist.twist.angular.vector.x
    rel_vel_msg.twist.angular.y = target_state.twist.twist.angular.vector.y-drone_state.twist.twist.angular.vector.y
    rel_vel_msg.twist.angular.z = target_state.twist.twist.angular.vector.z-drone_state.twist.twist.angular.vector.z    
    return rel_vel_msg

def compute_relative_pos_msg(target_state,drone_state):
    '''
    Function computes the relative position vector between the moving platform and the drone.
    '''
    p_x = target_state.pose.pose.position.x-drone_state.pose.pose.position.x
    p_y = target_state.pose.pose.position.y-drone_state.pose.pose.position.y
    p_z = target_state.pose.pose.position.z-drone_state.pose.pose.position.z
    
    #Determine relative rotation from moving platform to drone using quaterions
    #Initialize array for quaternions
    q_0 = [0,0,0,1]
    q_1 = [0,0,0,1]

    #Create q_0 and q_0_inv
    q_0[0] = target_state.pose.pose.orientation.x
    q_0[1] = target_state.pose.pose.orientation.y
    q_0[2] = target_state.pose.pose.orientation.z
    q_0[3] = target_state.pose.pose.orientation.w
    q_0_inv = quaternion_inverse(q_0)
    
    #Create q_1
    q_1[0] = drone_state.pose.pose.orientation.x
    q_1[1] = drone_state.pose.pose.orientation.y
    q_1[2] = drone_state.pose.pose.orientation.z
    q_1[3] = drone_state.pose.pose.orientation.w

    roll_wp,pitch_wp,yaw_wp = euler_from_quaternion((q_0[0],q_0[1],q_0[2],q_0[3]))
    roll_drone,pitch_drone,yaw_drone = euler_from_quaternion((q_1[0],q_1[1],q_1[2],q_1[3]))
    # print(f"yaw_wp,yaw_drone,wp_drone_yaw: {np.rad2deg(yaw_wp):02}, {np.rad2deg(yaw_drone):02}, {np.rad2deg(yaw_wp)-np.rad2deg(yaw_drone):02}")

    #Computate rotation quaternion which yields q_1 = q_r*q_0
    q_r = quaternion_multiply(q_1,q_0_inv)
    q_new = Quaternion()
    q_new.x = q_r[0]
    q_new.y = q_r[1]
    q_new.z = q_r[2]
    q_new.w = q_r[3]

    rel_pose_msg = PoseStamped()
    rel_pose_msg.header.frame_id = drone_frame
    rel_pose_msg.header.stamp = rospy.Time.now()
    rel_pose_msg.pose.position.x = p_x
    rel_pose_msg.pose.position.y = p_y
    rel_pose_msg.pose.position.z = p_z
    rel_pose_msg.pose.orientation = q_new

    msg = Float64MultiArray()
    msg.data = (180/np.pi)*np.array(euler_from_quaternion([q_new.x,q_new.y,q_new.z,q_new.w]))
    relative_rpy_publisher.publish(msg)
    return rel_pose_msg



def compute_state_msg(landing_simulation_object):
    '''
    Function takes as input an instance of the class LandingSimulationObject and produces a ROS msg
    based on the values that are currently stored in the class. It also performs a check if all
    sensor values are defined in the same coordinate frame.
    '''
    state = State()
    state.twist.twist.linear = Vector3Stamped()
    state.twist.twist.angular = Vector3Stamped()

    state.header.stamp = rospy.Time.now()

    #Check if all coordinate frames are the same
    ### Use this line if IMUs are used
    #if landing_simulation_object.pose.header.frame_id == landing_simulation_object.twist.header.frame_id and landing_simulation_object.pose.header.frame_id == landing_simulation_object.linear_acceleration.header.frame_id:
    if landing_simulation_object.pose.header.frame_id == landing_simulation_object.twist.header.frame_id:
        state.header.frame_id = landing_simulation_object.pose.header.frame_id
    else:
        rospy.logwarn("Coordinate frames are not the same for position, velocity and acceleration.")
        rospy.logwarn('Pose frame:')
        rospy.logwarn(landing_simulation_object.pose.header.frame_id)
        rospy.logwarn('Twist frame:')
        rospy.logwarn(landing_simulation_object.twist.header.frame_id)
        rospy.logwarn('Linear acceleration frame:')
        rospy.logwarn(landing_simulation_object.linear_acceleration.header.frame_id)
    state.pose = deepcopy(landing_simulation_object.pose)
    state.twist = deepcopy(landing_simulation_object.twist)
    return state

def compute_rpy_std_msg(q):
    msg = Float64MultiArray()
    msg.data = euler_from_quaternion([q.x,q.y,q.z,q.w])
    return msg


if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)

    drone_subscriber = rospy.Subscriber(sphinx_drone_topic[0],sphinx_drone_topic[1],read_drone)
    waypoint_subscriber = rospy.Subscriber(waypoint_topic[0],waypoint_topic[1],read_waypoint)

    #Init tf2
    tfBuffer = tf2_ros.Buffer()
    #time.sleep(10)
    listener = tf2_ros.TransformListener(tfBuffer)

    #Init rate
    rate = rospy.Rate(relative_info_hz)

    #commands to be executed as long as node is up
    rospy.loginfo("Start running node at "+str(relative_info_hz)+"hz")

    while not rospy.is_shutdown():
        try:
            #transform drone and moving platform position and velocity in target frame .                                    
            trans_world_to_drone_frame = tfBuffer.lookup_transform(drone_frame,wp_frame, rospy.Time())

            target_state_in_drone_frame.pose = tf2_geometry_msgs.do_transform_pose(target_state_in_wp_frame.pose,trans_world_to_drone_frame)
            target_state_in_drone_frame.pose.header.frame_id = drone_frame

            target_state_in_drone_frame.twist.twist.linear = tf2_geometry_msgs.do_transform_vector3(target_state_in_wp_frame.twist.twist.linear,trans_world_to_drone_frame)
            target_state_in_drone_frame.twist.twist.angular = tf2_geometry_msgs.do_transform_vector3(target_state_in_wp_frame.twist.twist.angular,trans_world_to_drone_frame)
            target_state_in_drone_frame.twist.header.frame_id = drone_frame

            drone_state_in_drone_frame.pose = tf2_geometry_msgs.do_transform_pose(drone_state_in_wp_frame.pose,trans_world_to_drone_frame)
            drone_state_in_drone_frame.pose.header.frame_id = drone_frame

            drone_state_in_drone_frame.twist.twist.linear = tf2_geometry_msgs.do_transform_vector3(drone_state_in_wp_frame.twist.twist.linear,trans_world_to_drone_frame)
            drone_state_in_drone_frame.twist.twist.angular = tf2_geometry_msgs.do_transform_vector3(drone_state_in_wp_frame.twist.twist.angular,trans_world_to_drone_frame)
            drone_state_in_drone_frame.twist.header.frame_id = drone_frame

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('transformation from gazebo model frame to drone frame threw an error')
            rate.sleep()
            continue

        #Publish the messages
        rel_vel_msg = compute_relative_vel_msg(target_state_in_drone_frame,drone_state_in_drone_frame)
        rel_vel_world_msg = compute_relative_vel_msg(target_state_in_wp_frame,drone_state_in_wp_frame)
        relative_vel_state_publisher.publish(rel_vel_msg)
        relative_vel_state_world_publisher.publish(rel_vel_world_msg)

        rel_pos_msg = compute_relative_pos_msg(target_state_in_drone_frame,drone_state_in_drone_frame)
        rel_pos_world_msg = compute_relative_pos_msg(target_state_in_wp_frame,drone_state_in_wp_frame)

        relative_pos_state_publisher.publish(rel_pos_msg)
        relative_pos_state_world_publisher.publish(rel_pos_world_msg)
        drone_state_msg = compute_state_msg(drone_state_in_drone_frame)
        drone_state_publisher.publish(drone_state_msg)
        drone_rpy_msg = compute_rpy_std_msg(drone_state_in_drone_frame.pose.pose.orientation)
        drone_rpy_publisher.publish(drone_rpy_msg)
        drone_state_world_msg = compute_state_msg(drone_state_in_wp_frame)
        drone_state_world_publisher.publish(drone_state_world_msg)
        target_msg = compute_state_msg(target_state_in_drone_frame)
        target_state_publisher.publish(target_msg)
        target_rpy_msg = compute_rpy_std_msg(target_state_in_drone_frame.pose.pose.orientation)
        target_rpy_publisher.publish(target_rpy_msg)
        target_world_msg = compute_state_msg(target_state_in_wp_frame)

        target_state_world_publisher.publish(target_world_msg)

        rate.sleep()