import subprocess
import threading
import rospy
from sphinx_with_gazebo.msg import Sphinx


#Parameters
node_name = "sphinx_interface_node"
publish_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/publish_hz',"10"))

#Topics
anafi_data_topic = ("/anafi/sphinx/drone_data",Sphinx)




class ReadSphinxData():
    def __init__(self):


        #publishers
        self.anafi_data_publisher = rospy.Publisher(anafi_data_topic[0],anafi_data_topic[1],queue_size=0)
    

        # The shell commands to be run
        self.command_worldPosition_x         = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.worldPosition.x'"
        self.command_worldPosition_y         = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.worldPosition.y'"
        self.command_worldPosition_z         = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.worldPosition.z'"
        self.command_worldLinearVelocity_x   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.worldLinearVelocity.x'"
        self.command_worldLinearVelocity_y   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.worldLinearVelocity.y'"
        self.command_worldLinearVelocity_z   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.worldLinearVelocity.z'"
        self.command_worldAttitude_x         = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.worldAttitude.x'"
        self.command_worldAttitude_y         = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.worldAttitude.y'"
        self.command_worldAttitude_z         = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.worldAttitude.z'"
        self.command_relativeLinearVelocity_x   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeLinearVelocity.x'"
        self.command_relativeLinearVelocity_y   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeLinearVelocity.y'"
        self.command_relativeLinearVelocity_z   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeLinearVelocity.z'"
        self.command_relativeAngularVelocity_x   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeAngularVelocity.x'"
        self.command_relativeAngularVelocity_y   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeAngularVelocity.y'"
        self.command_relativeAngularVelocity_z   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeAngularVelocity.z'"
        self.command_relativeLinearAcceleration_x   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeLinearAcceleration.x'"
        self.command_relativeLinearAcceleration_y   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeLinearAcceleration.y'"
        self.command_relativeLinearAcceleration_z   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeLinearAcceleration.z'"
        self.command_relativeAngularAcceleration_x   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeAngularAcceleration.x'"
        self.command_relativeAngularAcceleration_y   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeAngularAcceleration.y'"
        self.command_relativeAngularAcceleration_z   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.relativeAngularAcceleration.z'"
        self.command_timestamp   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'omniscient_anafi.timestamp'"
        self.command_gimbal_roll_anafi_motor_angle   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'gimbal_roll_anafi.motor_angle'"
        self.command_gimbal_pitch_anafi_motor_angle   = "tlm-data-logger -r 4 inet:127.0.0.1:9060 | grep --line-buffered 'gimbal_pitch_anafi.motor_angle'"
        self.value_worldPosition_x         = 0
        self.value_worldPosition_y         = 0
        self.value_worldPosition_z         = 0
        self.value_worldLinearVelocity_x   = 0
        self.value_worldLinearVelocity_y   = 0
        self.value_worldLinearVelocity_z   = 0
        self.value_worldAttitude_x         = 0
        self.value_worldAttitude_y         = 0
        self.value_worldAttitude_z         = 0
        self.value_relativeLinearVelocity_x   = 0
        self.value_relativeLinearVelocity_y   = 0
        self.value_relativeLinearVelocity_z   = 0
        self.value_relativeAngularVelocity_x   = 0
        self.value_relativeAngularVelocity_y   = 0
        self.value_relativeAngularVelocity_z   = 0
        self.value_relativeLinearAcceleration_x   = 0
        self.value_relativeLinearAcceleration_y   = 0
        self.value_relativeLinearAcceleration_z   = 0
        self.value_relativeAngularAcceleration_x   = 0
        self.value_relativeAngularAcceleration_y   = 0
        self.value_relativeAngularAcceleration_z   = 0
        self.value_timestamp = 0
        self.value_gimbal_roll_anafi_motor_angle = 0
        self.value_gimbal_pitch_anafi_motor_angle = 0
        return








    # Function to read process output
    def read_output(self,process,pattern):
        # print("pattern =",pattern)
        
        for line in iter(process.stdout.readline, b''):  # Read line by line
            latest_data = line.strip()  # Update with the latest line
            # Optionally print or log the latest data (for debugging)
            # print(f"Latest Data: {latest_data}")
            # print("latest_data =",latest_data)
            
            # print("split_string =",split_string)
            split_latest_data = latest_data.split(":")
            value = float(split_latest_data[1])
            # print("value =",pattern,":",value)

            if "." in pattern:
                setattr(self,"value_"+pattern.replace(".","_"),value)

            else:
                setattr(self,"value_"+pattern,value)



    def start_readers(self):

        # Start the process
        self.process_worldPosition_x       = subprocess.Popen(self.command_worldPosition_x         , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_worldPosition_y       = subprocess.Popen(self.command_worldPosition_y         , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_worldPosition_z       = subprocess.Popen(self.command_worldPosition_z         , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        self.process_worldLinearVelocity_x = subprocess.Popen(self.command_worldLinearVelocity_x   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_worldLinearVelocity_y = subprocess.Popen(self.command_worldLinearVelocity_y   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_worldLinearVelocity_z = subprocess.Popen(self.command_worldLinearVelocity_z   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        self.process_worldAttitude_x            = subprocess.Popen(self.command_worldAttitude_x              , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_worldAttitude_y            = subprocess.Popen(self.command_worldAttitude_y              , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_worldAttitude_z            = subprocess.Popen(self.command_worldAttitude_z              , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        self.process_relativeLinearVelocity_x = subprocess.Popen(self.command_relativeLinearVelocity_x   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_relativeLinearVelocity_y = subprocess.Popen(self.command_relativeLinearVelocity_y   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_relativeLinearVelocity_z = subprocess.Popen(self.command_relativeLinearVelocity_z   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        self.process_relativeAngularVelocity_x = subprocess.Popen(self.command_relativeAngularVelocity_x   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_relativeAngularVelocity_y = subprocess.Popen(self.command_relativeAngularVelocity_y   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_relativeAngularVelocity_z = subprocess.Popen(self.command_relativeAngularVelocity_z   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        self.process_relativeLinearAcceleration_x = subprocess.Popen(self.command_relativeLinearAcceleration_x   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_relativeLinearAcceleration_y = subprocess.Popen(self.command_relativeLinearAcceleration_y   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_relativeLinearAcceleration_z = subprocess.Popen(self.command_relativeLinearAcceleration_z   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        self.process_relativeAngularAcceleration_x = subprocess.Popen(self.command_relativeAngularAcceleration_x   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_relativeAngularAcceleration_y = subprocess.Popen(self.command_relativeAngularAcceleration_y   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.process_relativeAngularAcceleration_z = subprocess.Popen(self.command_relativeAngularAcceleration_z   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)  

        self.process_timestamp = subprocess.Popen(self.command_timestamp   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        self.process_gimbal_roll_anafi_motor_angle = subprocess.Popen(self.command_gimbal_roll_anafi_motor_angle   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)        
        self.process_gimbal_pitch_anafi_motor_angle = subprocess.Popen(self.command_gimbal_pitch_anafi_motor_angle   , shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)        


        # Run the output reader in a separate thread
        self.thread_worldPosition_x = threading.Thread(target=self.read_output, args=(self.process_worldPosition_x,"worldPosition_x",))
        self.thread_worldPosition_x.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_worldPosition_x.start()
        self.thread_worldPosition_y = threading.Thread(target=self.read_output, args=(self.process_worldPosition_y,"worldPosition_y",))
        self.thread_worldPosition_y.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_worldPosition_y.start()
        self.thread_worldPosition_z = threading.Thread(target=self.read_output, args=(self.process_worldPosition_z,"worldPosition_z",))
        self.thread_worldPosition_z.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_worldPosition_z.start()

        self.thread_worldLinearVelocity_x = threading.Thread(target=self.read_output, args=(self.process_worldLinearVelocity_x,"worldLinearVelocity_x",))
        self.thread_worldLinearVelocity_x.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_worldLinearVelocity_x.start()
        self.thread_worldLinearVelocity_y = threading.Thread(target=self.read_output, args=(self.process_worldLinearVelocity_y,"worldLinearVelocity_y",))
        self.thread_worldLinearVelocity_y.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_worldLinearVelocity_y.start()
        self.thread_worldLinearVelocity_z = threading.Thread(target=self.read_output, args=(self.process_worldLinearVelocity_z,"worldLinearVelocity_z",))
        self.thread_worldLinearVelocity_z.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_worldLinearVelocity_z.start()

        self.thread_worldAttitude_x = threading.Thread(target=self.read_output, args=(self.process_worldAttitude_x,"worldAttitude_x",))
        self.thread_worldAttitude_x.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_worldAttitude_x.start()
        self.thread_worldAttitude_y = threading.Thread(target=self.read_output, args=(self.process_worldAttitude_y,"worldAttitude_y",))
        self.thread_worldAttitude_y.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_worldAttitude_y.start()
        self.thread_worldAttitude_z = threading.Thread(target=self.read_output, args=(self.process_worldAttitude_z,"worldAttitude_z",))
        self.thread_worldAttitude_z.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_worldAttitude_z.start()

        self.thread_relativeLinearVelocity_x = threading.Thread(target=self.read_output, args=(self.process_relativeLinearVelocity_x,"relativeLinearVelocity_x",))
        self.thread_relativeLinearVelocity_x.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeLinearVelocity_x.start()
        self.thread_relativeLinearVelocity_y = threading.Thread(target=self.read_output, args=(self.process_relativeLinearVelocity_y,"relativeLinearVelocity_y",))
        self.thread_relativeLinearVelocity_y.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeLinearVelocity_y.start()
        self.thread_relativeLinearVelocity_z = threading.Thread(target=self.read_output, args=(self.process_relativeLinearVelocity_z,"relativeLinearVelocity_z",))
        self.thread_relativeLinearVelocity_z.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeLinearVelocity_z.start()

        self.thread_relativeAngularVelocity_x = threading.Thread(target=self.read_output, args=(self.process_relativeAngularVelocity_x,"relativeAngularVelocity_x",))
        self.thread_relativeAngularVelocity_x.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeAngularVelocity_x.start()
        self.thread_relativeAngularVelocity_y = threading.Thread(target=self.read_output, args=(self.process_relativeAngularVelocity_y,"relativeAngularVelocity_y",))
        self.thread_relativeAngularVelocity_y.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeAngularVelocity_y.start()
        self.thread_relativeAngularVelocity_z = threading.Thread(target=self.read_output, args=(self.process_relativeAngularVelocity_z,"relativeAngularVelocity_z",))
        self.thread_relativeAngularVelocity_z.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeAngularVelocity_z.start()

        self.thread_relativeLinearAcceleration_x = threading.Thread(target=self.read_output, args=(self.process_relativeLinearAcceleration_x,"relativeLinearAcceleration_x",))
        self.thread_relativeLinearAcceleration_x.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeLinearAcceleration_x.start()
        self.thread_relativeLinearAcceleration_y = threading.Thread(target=self.read_output, args=(self.process_relativeLinearAcceleration_y,"relativeLinearAcceleration_y",))
        self.thread_relativeLinearAcceleration_y.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeLinearAcceleration_y.start()
        self.thread_relativeLinearAcceleration_z = threading.Thread(target=self.read_output, args=(self.process_relativeLinearAcceleration_z,"relativeLinearAcceleration_z",))
        self.thread_relativeLinearAcceleration_z.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeLinearAcceleration_z.start()

        self.thread_relativeAngularAcceleration_x = threading.Thread(target=self.read_output, args=(self.process_relativeAngularAcceleration_x,"relativeAngularAcceleration_x",))
        self.thread_relativeAngularAcceleration_x.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeAngularAcceleration_x.start()
        self.thread_relativeAngularAcceleration_y = threading.Thread(target=self.read_output, args=(self.process_relativeAngularAcceleration_y,"relativeAngularAcceleration_y",))
        self.thread_relativeAngularAcceleration_y.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeAngularAcceleration_y.start()
        self.thread_relativeAngularAcceleration_z = threading.Thread(target=self.read_output, args=(self.process_relativeAngularAcceleration_z,"relativeAngularAcceleration_z",))
        self.thread_relativeAngularAcceleration_z.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_relativeAngularAcceleration_z.start()

        self.thread_timestamp = threading.Thread(target=self.read_output, args=(self.process_timestamp,"timestamp",))
        self.thread_timestamp.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_timestamp.start()

        self.thread_gimbal_roll_anafi_motor_angle = threading.Thread(target=self.read_output, args=(self.process_gimbal_roll_anafi_motor_angle,"gimbal_roll_anafi.motor_angle",))
        self.thread_gimbal_roll_anafi_motor_angle.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_gimbal_roll_anafi_motor_angle.start()

        self.thread_gimbal_pitch_anafi_motor_angle = threading.Thread(target=self.read_output, args=(self.process_gimbal_pitch_anafi_motor_angle,"gimbal_pitch_anafi.motor_angle",))
        self.thread_gimbal_pitch_anafi_motor_angle.daemon = True  # Ensures the thread exits if the main program exits
        self.thread_gimbal_pitch_anafi_motor_angle.start()
        return

    def terminate_readers(self):
        print("Beginning termination of readers!")
        self.process_worldPosition_x.terminate()  # Terminate the subprocess
        self.thread_worldPosition_x.join()
        self.process_worldPosition_y.terminate()  # Terminate the subprocess
        self.thread_worldPosition_y.join()
        self.process_worldPosition_z.terminate()  # Terminate the subprocess
        self.thread_worldPosition_z.join()

        self.process_worldLinearVelocity_x.terminate()  # Terminate the subprocess
        self.thread_worldLinearVelocity_x.join()
        self.process_worldLinearVelocity_y.terminate()  # Terminate the subprocess
        self.thread_worldLinearVelocity_y.join()
        self.process_worldLinearVelocity_z.terminate()  # Terminate the subprocess
        self.thread_worldLinearVelocity_z.join()

        self.process_worldAttitude_x.terminate()  # Terminate the subprocess
        self.thread_worldAttitude_x.join()
        self.process_worldAttitude_y.terminate()  # Terminate the subprocess
        self.thread_worldAttitude_y.join()
        self.process_worldAttitude_z.terminate()  # Terminate the subprocess
        self.thread_worldAttitude_z.join()

        self.process_relativeLinearVelocity_x.terminate()  # Terminate the subprocess
        self.thread_relativeLinearVelocity_x.join()
        self.process_relativeLinearVelocity_y.terminate()  # Terminate the subprocess
        self.thread_relativeLinearVelocity_y.join()
        self.process_relativeLinearVelocity_z.terminate()  # Terminate the subprocess
        self.thread_relativeLinearVelocity_z.join()

        self.process_relativeAngularVelocity_x.terminate()  # Terminate the subprocess
        self.thread_relativeAngularVelocity_x.join()
        self.process_relativeAngularVelocity_y.terminate()  # Terminate the subprocess
        self.thread_relativeAngularVelocity_y.join()
        self.process_relativeAngularVelocity_z.terminate()  # Terminate the subprocess
        self.thread_relativeAngularVelocity_z.join()

        self.process_relativeLinearAcceleration_x.terminate()  # Terminate the subprocess
        self.thread_relativeLinearAcceleration_x.join()
        self.process_relativeLinearAcceleration_y.terminate()  # Terminate the subprocess
        self.thread_relativeLinearAcceleration_y.join()
        self.process_relativeLinearAcceleration_z.terminate()  # Terminate the subprocess
        self.thread_relativeLinearAcceleration_z.join()

        self.process_relativeAngularAcceleration_x.terminate()  # Terminate the subprocess
        self.thread_relativeAngularAcceleration_x.join()
        self.process_relativeAngularAcceleration_y.terminate()  # Terminate the subprocess
        self.thread_relativeAngularAcceleration_y.join()
        self.process_relativeAngularAcceleration_z.terminate()  # Terminate the subprocess
        self.thread_relativeAngularAcceleration_z.join()

        self.process_timestamp.terminate()  # Terminate the subprocess
        self.thread_timestamp.join()

        self.process_gimbal_roll_anafi_motor_angle.terminate()  # Terminate the subprocess
        self.thread_gimbal_roll_anafi_motor_angle.join()

        self.process_gimbal_pitch_anafi_motor_angle.terminate()  # Terminate the subprocess
        self.thread_gimbal_pitch_anafi_motor_angle.join()

        print("Completed termination of readers!")

        return

    def publish_sphinx_msg(self):
        msg = Sphinx()
        msg.posX = self.value_worldPosition_x
        msg.posY = self.value_worldPosition_y
        msg.posZ = self.value_worldPosition_z

        msg.velXENU = self.value_worldLinearVelocity_x
        msg.velYENU = self.value_worldLinearVelocity_y
        msg.velZENU = self.value_worldLinearVelocity_z

        msg.attitudeX = self.value_worldAttitude_x
        msg.attitudeY = self.value_worldAttitude_y
        msg.attitudeZ = self.value_worldAttitude_z

        msg.velXABC = self.value_relativeLinearVelocity_x
        msg.velYABC = self.value_relativeLinearVelocity_y
        msg.velZABC = self.value_relativeLinearVelocity_z

        msg.angVelXABC = self.value_relativeLinearVelocity_x
        msg.angVelYABC = self.value_relativeLinearVelocity_y
        msg.angVelZABC = self.value_relativeLinearVelocity_z

        msg.accXABC = self.value_relativeLinearAcceleration_x
        msg.accYABC = self.value_relativeLinearAcceleration_y
        msg.accZABC = self.value_relativeLinearAcceleration_z

        msg.angAccXABC = self.value_relativeAngularAcceleration_x
        msg.angAccYABC = self.value_relativeAngularAcceleration_y
        msg.angAccZABC = self.value_relativeAngularAcceleration_z

        msg.timeStamp = self.value_timestamp

        msg.gimbalRollMotorAngle = self.value_gimbal_roll_anafi_motor_angle
        msg.gimbalPitchMotorAngle = self.value_gimbal_pitch_anafi_motor_angle

        # print(msg)

        self.anafi_data_publisher.publish(msg)
        return






if __name__ == "__main__":


    rospy.init_node(node_name)

    db = ReadSphinxData()
    db.start_readers()

    rate = rospy.Rate(publish_hz)

    rospy.on_shutdown(db.terminate_readers)

    while not rospy.is_shutdown():
        # print("---")
        # print(db.value_worldPosition_x)
        # print(db.value_worldPosition_y)
        # print(db.value_worldPosition_z)
        # print(db.value_worldLinearVelocity_x)
        # print(db.value_worldLinearVelocity_y)
        # print(db.value_worldLinearVelocity_z)
        # print(db.value_worldAttitude_x)
        # print(db.value_worldAttitude_y)
        # print(db.value_worldAttitude_z)




        db.publish_sphinx_msg()
        rate.sleep()
    