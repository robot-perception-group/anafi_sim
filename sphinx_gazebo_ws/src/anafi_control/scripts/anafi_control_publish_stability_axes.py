import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped
from geometry_msgs.msg import PoseStamped

node_name = 'anafi_control_publish_stability_axes_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','anafi')

#Odometry topic of the drone
pose_topic = ("/"+drone_name+"/drone/pose_enu",PoseStamped)

#Initialize node
rospy.init_node(node_name)
tfBuffer = Buffer()
listener = TransformListener(tfBuffer)
br = TransformBroadcaster()

def publish_stability_axes_as_tf_frame(parent_frame,br,phi,theta,psi,x,y,z):
    """Function publishes the stability axes frame w.r.t. the world frame."""
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = drone_name + "/stability_axes_anafi_control"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    q = quaternion_from_euler(0, 0, psi)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]    
    br.sendTransform(t)
    print(t.header.stamp,"Anafi control: Stability axes frame transformation sent.")
    return

def read_pose_msg(msg): 
    """Function reads the Sphinx message from the sphinx interface node.
    """
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    phi,theta,psi = euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])



    # phi = msg.attitudeY
    # theta = msg.attitudeX
    # psi = -msg.attitudeZ
    
    #Publish frame transformation from odometry_sensor1 to stability frame
    publish_stability_axes_as_tf_frame('world',br, phi, theta, psi, x,y,z)
    return

sphinx_subscriber = rospy.Subscriber(pose_topic[0],pose_topic[1],read_pose_msg)
rospy.loginfo("publisher node for stability axes frame started ")
rospy.spin()
