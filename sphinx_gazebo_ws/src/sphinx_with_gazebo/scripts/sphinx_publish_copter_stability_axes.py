'''
Function publishes the stability axes as tf frame. The stability axes xy-plane is always parallel to earth surface but the x-axes is aligned with the x-axis of the body-fixed
frame.
For this purpose, the odometry topic of the drone that is published by the rotorS package is read and whenever a message is received a the stability axes is published
as tf frame.
'''

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped
from sphinx_with_gazebo.msg import Sphinx

node_name = 'sphinx_stability_frame_publisher_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name')

#Odometry topic of the drone
sphinx_topic = ("/anafi/sphinx/drone_data",Sphinx)

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
    t.child_frame_id = drone_name + "/stability_axes"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    q = quaternion_from_euler(0, 0, psi)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]    
    br.sendTransform(t)
    print(t.header.stamp,"Transform sent.")
    return

def read_sphinx_msg(msg): 
    """Function reads the Sphinx message from the sphinx interface node.
    """
    x = msg.posX
    y = msg.posY
    z = msg.posZ

    phi = msg.attitudeX
    theta = msg.attitudeY
    psi = msg.attitudeZ

    # phi = msg.attitudeY
    # theta = msg.attitudeX
    # psi = -msg.attitudeZ
    
    #Publish frame transformation from odometry_sensor1 to stability frame
    publish_stability_axes_as_tf_frame('world',br, phi, theta, psi, x,y,z)
    return

sphinx_subscriber = rospy.Subscriber(sphinx_topic[0],sphinx_topic[1],read_sphinx_msg)
rospy.loginfo("publisher node for stability axes frame started ")
rospy.spin()
