#!/usr/bin/env python
import rospy
import yaml
import math
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped, Vector3

class imu_calibration():
    def __init__(self):
        self.config_str_acc = "**** Configuring Accelerometeres ****"
        self.config_str_gyro = "**** Configuring Gyroscopes ****"
        self.config_str_mag = "**** Configuring Magnetometer ****"
        self.config_str_acc_step = " (desierd positive) should be alligned with the direction of the gravity vector"
        self.config_str_mag_step = " (desierd positive) should point towards the north"
        self.config_str_gyro_step = " (desierd positive) should be rotated in a positive direction about the axis"
        ## thresholds
        self.acc_threshold = 11.0
        self.gyro_threshold = 1.5
        self.mag_threshold = 0.8

        ##correction info 
        self.acc_direction_correction_list = [0]*3
        self.gyro_direction_correction_list = [0]*3
        self.mag_direction_correction_list = [0]*3
        ## axis mapping
        self.acc_vector_mapping_direction_correction_list = [0]*3
        self.gyro_vector_mapping_direction_correction_list = [0]*3
        self.mag_vector_mapping_direction_correction_list = [0]*3
        ##updated current values
        self.acc_current_list = [0]*3
        self.gyro_current_list = [0]*3
        self.mag_current_list = [0]*3
        
        # get parameters
        imu_data_tn = rospy.get_param('~imu_data_topic_name', "/sensor/imu/data")
        mag_data_tn = rospy.get_param('~mag_data_topic_name', "/sensor/imu/mag")
        mag_data_vector_tn = rospy.get_param('~mag_data_vector_topic_name', "/sensor/imu/mag_vector")
        self.imu_name = rospy.get_param('~imu_name', "Unknown_imu")
        self.absolute_path = rospy.get_param('~package_path', "")

        # create subscribers
        self.imu_subs = rospy.Subscriber(imu_data_tn, Imu, self.Imu_update)
        self.mag_subs = rospy.Subscriber(mag_data_tn, MagneticField, self.mag_update)
        self.mag_vector_subs = rospy.Subscriber(mag_data_vector_tn, Vector3Stamped, self.mag_vector_update) # should not be needed

    def Imu_update(self, msg):

        #print ("in call back IMU")
        acc_vector = msg.linear_acceleration
        self.acc_current_list[0] = acc_vector.x
        self.acc_current_list[1] = acc_vector.y
        self.acc_current_list[2] = acc_vector.z
        
        gyro_vector = msg.angular_velocity
        self.gyro_current_list[0] = gyro_vector.x
        self.gyro_current_list[1] = gyro_vector.y
        self.gyro_current_list[2] = gyro_vector.z
        

    def mag_update(self, msg):
        #print ("in call back MAG")
        mag_vector = msg.magnetic_field
        v = [None]*3
        v[0] = msg.magnetic_field.x
        v[1] = msg.magnetic_field.y
        v[2] = msg.magnetic_field.z
        magnitude = math.sqrt(sum(v[i]*v[i] for i in range(len(v))))
        for i, value in enumerate(v):
            v[i] = v[i]/magnitude 
            self.mag_current_list[i] = v[i]

    def mag_vector_update(self, msg):
        mag_vector = msg.vector
        self.mag_current_list[0] = mag_vector.x
        self.mag_current_list[1] = mag_vector.y
        self.mag_current_list[2] = mag_vector.z

    def acc_config(self):
        print (self.config_str_acc)
        print ("x" + self.config_str_acc_step)
        self.axis_check(self.acc_current_list, self.acc_direction_correction_list, self.acc_vector_mapping_direction_correction_list, self.acc_threshold, 0)
        print ("y" + self.config_str_acc_step)
        self.axis_check(self.acc_current_list, self.acc_direction_correction_list, self.acc_vector_mapping_direction_correction_list, self.acc_threshold, 1)
        print ("z" + self.config_str_acc_step)
        self.axis_check(self.acc_current_list, self.acc_direction_correction_list, self.acc_vector_mapping_direction_correction_list, self.acc_threshold, 2)
        
    def gyro_config(self):
        print (self.config_str_gyro)
        print ("x" + self.config_str_gyro_step)
        self.axis_check(self.gyro_current_list, self.gyro_direction_correction_list, self.gyro_vector_mapping_direction_correction_list, self.gyro_threshold, 0)
        print ("y" + self.config_str_gyro_step)
        self.axis_check(self.gyro_current_list, self.gyro_direction_correction_list, self.gyro_vector_mapping_direction_correction_list, self.gyro_threshold, 1)
        print ("z" + self.config_str_gyro_step)
        self.axis_check(self.gyro_current_list, self.gyro_direction_correction_list, self.gyro_vector_mapping_direction_correction_list, self.gyro_threshold, 2)

    def mag_config(self):
        print (self.config_str_mag)
        print ("x" + self.config_str_mag_step)
        self.axis_check(self.mag_current_list, self.mag_direction_correction_list, self.mag_vector_mapping_direction_correction_list, self.mag_threshold, 0)
        print ("y" + self.config_str_mag_step)
        self.axis_check(self.mag_current_list, self.mag_direction_correction_list, self.mag_vector_mapping_direction_correction_list, self.mag_threshold, 1)
        print ("z" + self.config_str_mag_step)
        self.axis_check(self.mag_current_list, self.mag_direction_correction_list, self.mag_vector_mapping_direction_correction_list, self.mag_threshold, 2)
        
    def check_and_return_sign(self, measured_list, threshold, axis):
        axis_dic = {0:"x",1:"y",2:"z"}
        index = -1
        direction = 0
        mapping = 0
        # find index of element
        for i, value in enumerate(measured_list):
            if abs(value) >= threshold:
                index = i
                break
        # check direction
        if measured_list[i] >= threshold:
            direction = 1 # we are in the proper direction already
        else:
            direction = -1
        # check axis
        if index != axis: # the index does not match the axis == we detected the requierd quantity on a diffrent axis, meaning mapping will have to take place
            print ("axis mapping was needed for axis :"+axis_dic[axis]+"    "+axis_dic[axis]+" --> "+str(direction)+" * "+axis_dic[index])
        else:
            print ("NO axis mapping was needed for axis :"+axis_dic[axis]+"    "+axis_dic[axis]+ " --> "+str(direction)+" * "+axis_dic[index])
        mapping = index
        return [mapping, direction]
    
    def axis_check(self,sensor_current, sensor_correction_direction, sensor_correction_mapping ,sensor_threshold,axis):
        raw_input ("press enter to continue")
        while (all(abs(entry) <= sensor_threshold for entry in sensor_current) and not rospy.is_shutdown()):
            print sensor_current
            rospy.sleep(0.1)
        sensor_measured_list = sensor_current
        print ("measured values =: "+str(sensor_measured_list))
        print ("Threshold is =: "+str(sensor_threshold))
        [mapping, sign] = self.check_and_return_sign(sensor_measured_list, sensor_threshold, axis)
        sensor_correction_mapping[axis] = mapping
        sensor_correction_direction[axis] = sign
        
    def yaml_write(self, name):
        
        name_list = ["acc_direction", "acc_mapping",  "gyro_direction", "gyro_mapping", "mag_direction", "mag_mapping"]
        value_list = [self.acc_direction_correction_list, self.acc_vector_mapping_direction_correction_list, self.gyro_direction_correction_list, 
                      self.gyro_vector_mapping_direction_correction_list, self.mag_direction_correction_list, self.mag_vector_mapping_direction_correction_list]
        # simple objects
        output_list_simple = []
        # iterate on the 3 lists together
        for name, value in zip(name_list, value_list):
            output_list_simple.append({name:value})
        
        # python objects
#         output_list = [0]*6
#         for list_entry, name, value in zip(output_list,name_list, value_list):
#             list_entry = self.write_sub(value, name)
        
        #physical dump to file
        with open(self.absolute_path+self.imu_name+"_coordinate_config.yaml", 'w+') as f:
            f.write('# autogenerated: Do not edit #\n')
#             f.write('# Complex representation #\n')
#             f.write(yaml.dump(output_list))
#             f.write('# Simple representation #\n')
            for entry in output_list_simple:
                f.write(yaml.dump(entry))
        x = yaml.load(file(self.absolute_path+self.imu_name+"_coordinate_config.yaml"))
        print (x)
        
    def write_sub(self, vector, parameter):
        output = Vector3Stamped()
        output.header.frame_id = parameter
        output.vector.x = vector[0]
        output.vector.y = vector[1]
        output.vector.z = vector[2]
        return output
    
    def write_simple_sub(self, vector, parameter):
        output = parameter+" "
        output = output + str(vector)
        return output
#         file.write(yaml.dump(output))
            
#         def printtt(self):
#             print "HIHIHIHIHHIHHI"
#         yaml.dump(output, open(self.imu_name+"_config.yaml", "w"))
#         a = 1
#         self.gyro_direction_correction_list = [0]*3
#         self.mag_direction_correction_list = [0]*3
#         ## axis mapping
#         self.acc_vector_mapping_direction_correction_list = [0]*3
#         self.gyro_vector_mapping_direction_correction_list = [0]*3
#         self.mag_vector_mapping_direction_correction_list = [0]*3
#             f.write(yaml.dump())

if __name__=='__main__':
    rospy.init_node("coordinate_system_calib_node")
    calib_object = imu_calibration()
    calib_object.acc_config()
    calib_object.gyro_config()
    calib_object.mag_config()
    calib_object.yaml_write(calib_object.imu_name)
    print("Please exit the program")
    rospy.spin()
