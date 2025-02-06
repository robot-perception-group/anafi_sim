'''
Script contains functions that are generally useful to realize the training environment or to perform basic tasks.
'''
import rospy
import rosgraph
import os
import csv
import time
from gazebo_msgs.msg import ContactsState
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_matrix,quaternion_from_euler
import casadi as cs
import scipy.linalg



def get_publisher(topic_path:str, msg_type, **kwargs):
    ''' Function gets a publisher and returns an error if the publisher is not up. Source https://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/?answer=367990#post-id-367990.'''
    pub = rospy.Publisher(topic_path, msg_type, **kwargs)
    num_subs = len(_get_subscribers(topic_path))
    for i in range(10):
        num_cons = pub.get_num_connections()
        if num_cons == num_subs:
            # print("Got publisher",topic_path,"with ROS_MASTER_URI =",os.environ["ROS_MASTER_URI"],",GAZEBO_MASTER_URI =",os.environ["GAZEBO_MASTER_URI"],",ROS_IP =",os.environ["ROS_IP"])
            return pub
        time.sleep(0.1)
    raise RuntimeError("Failed to get publisher ",topic_path)

def _get_subscribers(topic_path:str):
    '''Function gets the list of subscribers to a topic. Source: https://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/?answer=367990#post-id-367990. '''
    ros_master = rosgraph.Master('/rostopic')
    topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
    state = ros_master.getSystemState()
    subs = []
    for sub in state[1]:
        if sub[0] == topic_path:
            subs.extend(sub[1])
    return subs



def write_data_to_csv(file_path:str,row_list:list,verbose:bool = False):
    """Functions writes a list of values as a row to a .csv file."""
    with open(file_path,'a+') as f:
            writer = csv.writer(f)
            writer.writerow(row_list)
            if verbose:
                print("added csv entry ",row_list,"to",file_path)
    return


def create_log_dir_path(parent_folder_path:str,tb_log_name:str):
    """Function determines a unique path where the log data should be stored within a parent folder. If a log with the same name but different log id exists, it returns a path to a subsequent folder with an id incremented by 1. """
    subfolder_index = int(0)
    for i in os.listdir(parent_folder_path):
        #Create absolute paths of all elements in parent_folder
        sub_path = os.path.join(parent_folder_path,i)

        #Iterate through all the sub element paths. If one is a directory that contains the string tb_log_name, execute the if clause
        if os.path.isdir(sub_path) and tb_log_name in i:
            #Get name of subfolder 
            subfolder_name = os.path.basename(os.path.normpath(sub_path))

            #get last segment of string after splitting at underscore and convert to int. This is the index number created by the training algorithm
            index = int(subfolder_name.split("_")[-1])
            if index > subfolder_index:
                subfolder_index = index
    
    #Compose path to logging directory
    log_dir_path = os.path.join(parent_folder_path,tb_log_name+"_"+str(subfolder_index + 1) )
    print("Data logging directory: ",log_dir_path)
    return log_dir_path

def create_log_dir(dir_path:str):
    if not os.path.exists(dir_path):
        try:
            os.makedirs(dir_path)
            print("Successfully created directory",dir_path)
        except OSError as e:
            print(f"Error creating folder '{dir_path}': {e}")
    else:
        print(f"Folder '{dir_path}' already exists. No new directory created.")
    return

def read_data_from_file_as_numpy(file_path,denominator = ','):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    data = []
    for line in lines:
        row = [float(value) for value in line.strip().split(denominator)]
        data.append(row)
    data_array = np.array(data)
    return data_array

def print_progress_bar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
        
        Based on https://stackoverflow.com/questions/3173320/text-progress-bar-in-terminal-with-block-characters
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()

def read_values_as_list(input = ""):
    with open(input, 'r') as file:
        # Read all lines from the file
        lines = file.readlines()

    # Split each line by spaces to create a list of values for each line
    values = []
    for line in lines:
        values.extend(line.split())

    # Convert each value to float
    values = [float(val) for val in values]
    return values
