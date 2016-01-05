#!/usr/bin/env python
import rospy
import yaml
import operator
import math
from geometry_msgs.msg import Vector3Stamped

from sensor_msgs.msg import Imu, MagneticField
from enable.savage.svg.css.values import length
class imu_sensor_calibration():
    def __init__(self):
        self.config_str_acc = "**** Configuring Accelerometer ****"
        self.config_str_gyro = "**** Configuring Gyroscopes ****"
#         self.config_str_mag = "**** Configuring Magnetometer ****"
        self.config_str_acc_step_positive = " (desired positive) should be aligned with the direction of the gravity vector"
        self.config_str_acc_step_negative = " (desired negative) should be aligned with the direction of the gravity vector"
#         self.config_str_mag_step = " (desired positive) should point towards the north"
        self.config_str_gyro_step = " set the IMU somewhere where it wont move and leave it till calibration is complete, press enter to start"
        self.config_str_gyro_error = " The imu has probably been  moved, the test will restart. \n threshold is: "
        self.config_str_acc_error = " The imu has probably been  moved, the measuerment will be ignored. \n threshold is: "
        
        ## thresholds
        self.acc_threshold = 11.0   #[m*s^-2]
        self.gyro_threshold = 1.5/180*math.pi #[input in deg then converted to rad]
        self.gyro_sample_number = 1000 # number of gyroscope measurements used
        
        # Read parameters of ros param server
        self.imu_name = rospy.get_param('~imu_name', "Unknown_imu")
        self.imu_data_tn = rospy.get_param('~imu_data_topic_name', "/sensor/imu/data")
        self.mag_data_tn = rospy.get_param('~mag_data_topic_name', "/sensor/imu/mag")
        self.absolute_path = rospy.get_param('~package_path', "")
        self.gravity = 9.81
        
        ## calibration variables
        self.gyro_correction_list = [0]*3 # used for gyro offset to center gyros to zero when not moving
        self.acc_correction_offset_list = [0]*3 # used to compensate unequal accelerometer measurements in the same direction with different signs (hysteresis) finds the new zero
        self.acc_correction_scaling_list = [0]*3 # used to rescale accelerometer values to the desired values
        self.gyro_readings = []#[[None]*self.gyro_sample_number,[None]*self.gyro_sample_number,[None]*self.gyro_sample_number]
        self.min_acc_readings = [0]*3
        self.max_acc_readings = [0]*3
        ## variables
        self.gyro_callback_flag = False
        self.gyro_retries = 0
        self.gyro_retries_limit = 5
        self.acc_axis_current = -1
        

    def calibrate_gyros(self):
        print self.config_str_gyro
        raw_input(self.config_str_gyro_step)
        self.gyro_readings = []
        self.gyro_callback_flag = True
        imu_subs = rospy.Subscriber(self.imu_data_tn, Imu, self.gyro_call_back)
        while (len(self.gyro_readings)<self.gyro_sample_number and self.gyro_callback_flag):
            pass#do nothing
        if self.gyro_callback_flag == False and self.gyro_retries < self.gyro_retries_limit:
            print "retrying gyro calibration"
            raw_input("press enter to retry \n")
            self.gyro_retries+=1
            imu_subs.unregister()
            self.calibrate_gyros()
        elif self.gyro_retries >= self.gyro_retries_limit:
            return False
        elif self.gyro_callback_flag == True:
            gyro_readings_sum = [0]*3 # x,y,z
            for index_large_list, entry in enumerate(self.gyro_readings):
#                 print "entry"+str(entry)
                for index_inside_entry, entry_component in enumerate(entry):
                    gyro_readings_sum[index_inside_entry] += entry_component
#                     print "index_large_list: "+str(index_large_list)
#                     print "index_inside_entry: "+str(index_inside_entry)
#                     print "entry_component: "+str(entry_component)
            gyro_readings_average = [0]*3
            for index, sum in enumerate(gyro_readings_sum):
                gyro_readings_average[index] = sum/len(self.gyro_readings)
            self.gyro_correction_list = gyro_readings_average
            print "len of gyro readings : "+str(len(self.gyro_readings))
            print "gyro correction data\n x    y    z \n"+str(self.gyro_correction_list)
            imu_subs.unregister()
            return True
        
    def gyro_call_back(self,imu_msg):
        gyro_vector_xyz = imu_msg.angular_velocity
        gyro_vector = [0]*3
        gyro_vector[0] = gyro_vector_xyz.x
        gyro_vector[1] = gyro_vector_xyz.y
        gyro_vector[2] = gyro_vector_xyz.z
        if len(self.gyro_readings)==0:
            self.gyro_readings.append(gyro_vector)
            self.gyro_callback_flag = True
            return True 
        else:
            reading_diff = map(operator.sub, gyro_vector,self.gyro_readings[-1])
            self.gyro_readings.append(gyro_vector)
            if any(abs(entry)>self.gyro_threshold for entry in reading_diff):
                print self.config_str_gyro_error + str((self.gyro_threshold)*180/math.pi)+ "deg.s^-1"
                print self.gyro_readings[len(self.gyro_readings)-2]#before current reading
                print self.gyro_readings[-1]# current reading
                self.gyro_callback_flag = False
                return False
            else:
                self.gyro_callback_flag = True
            
        
    def calibrate_accelerometer(self, axis_name, direction, axis_index):
        print self.config_str_acc
        self.acc_axis_current = axis_index
        if direction > 0:
            self.max_acc_readings[axis_index] = 0
            raw_input(self.config_str_acc_step_positive)
            
        else:
            self.min_acc_readings[axis_index] = 0
            raw_input(self.config_str_acc_step_negative)
        
        print "current axis is: "+axis_name
        imu_subs = rospy.Subscriber(self.imu_data_tn, Imu, self.accelerometer_call_back)#, axis_direction = direction)#callback_args = direction)
        i = raw_input("press 'r' to repeat ,press space to end calibration of axis "+axis_name)
        imu_subs.unregister()
        if (str(i) == "r"):
            self.calibrate_accelerometer(axis_name, direction, axis_index)
        elif(str(i) == " "):
            self.acc_correction_offset_list[axis_index] = (self.min_acc_readings[axis_index]+self.max_acc_readings[axis_index])/2 #mean value
            self.acc_correction_scaling_list[axis_index] = self.gravity/(self.max_acc_readings[axis_index]-self.acc_correction_offset_list[axis_index])
            print "calibration of axis %s yields:"%axis_name
            print "acc_offset is : "+str(self.acc_correction_offset_list[axis_index])
            print "acc_scaling is : "+str(self.acc_correction_scaling_list[axis_index])
        else:
            print ("********** INCORRECT INPUT *************, repeating measurement")
            self.calibrate_accelerometer(axis_name, direction, axis_index)
        return True
                
    def accelerometer_call_back(self,imu_msg):
        acc_vector_xyz = imu_msg.linear_acceleration
        axis_index = self.acc_axis_current
        if axis_index == 0:
            acc_value = acc_vector_xyz.x
        elif axis_index == 1:
            acc_value = acc_vector_xyz.y
        elif axis_index == 2:
            acc_value = acc_vector_xyz.z
        else:
            print "BAD acceleration AXIS DETECTED"
            return False
        
        if acc_value > abs(self.acc_threshold):
            print self.config_str_acc_error + str(self.acc_threshold) +"\n"
            print "measured value was :"+str(acc_value)
            return False
        
        if acc_value > self.max_acc_readings[axis_index]:
            self.max_acc_readings[axis_index] = acc_value
            
        if acc_value < self.min_acc_readings[axis_index]:
            self.min_acc_readings[axis_index] = acc_value
            
        print ("current value: %6.2f  min_value: %6.2f  max_value: %6.2f "%(acc_value ,self.min_acc_readings[axis_index], self.max_acc_readings[axis_index]) ,"   press 'r' to repeat ,press space to end calibration")
        
    def calib_check(self):
        imu_subs =  rospy.Subscriber(self.imu_data_tn, Imu, self.calib_check_call_back)
        i = raw_input("press 'r' to exit ,press space to end calibration and write to .yaml file")
        imu_subs.unregister()
        self.yaml_file_dump()
        print "Gyro offsets x:%10.5f  y:%10.5f  z:%10.5f"%(self.gyro_correction_list[0],self.gyro_correction_list[1],self.gyro_correction_list[2])
        print "accelerometer offsets x:%10.5f  y:%10.5f  z:%10.5f"%(self.acc_correction_offset_list[0],self.acc_correction_offset_list[1],self.acc_correction_offset_list[2])
        print "accelerometer scaling x:%10.5f  y:%10.5f  z:%10.5f"%(self.acc_correction_scaling_list[0],self.acc_correction_scaling_list[1],self.acc_correction_scaling_list[2])
        
    def calib_check_call_back(self,imu_msg):
        acc_vector_xyz = imu_msg.linear_acceleration
        acc_vector = [0]*3
        acc_vector[0] = (acc_vector_xyz.x - self.acc_correction_offset_list[0]) * self.acc_correction_scaling_list[0]
        acc_vector[1] = (acc_vector_xyz.y - self.acc_correction_offset_list[1]) * self.acc_correction_scaling_list[1]
        acc_vector[2] = (acc_vector_xyz.z - self.acc_correction_offset_list[2]) * self.acc_correction_scaling_list[2]
        
        gyro_vector_xyz = imu_msg.angular_velocity
        gyro_vector = [0]*3
        gyro_vector[0] = gyro_vector_xyz.x - self.gyro_correction_list[0]
        gyro_vector[1] = gyro_vector_xyz.y - self.gyro_correction_list[1]
        gyro_vector[2] = gyro_vector_xyz.z - self.gyro_correction_list[2]
        
        print ("Gyroscope readings x:%6.2f  y:%6.2f  z:%6.2f "%(gyro_vector[0],gyro_vector[1],gyro_vector[2]))
        print ("Accelerometer readings x:%6.2f  y:%6.2f  z:%6.2f "%(acc_vector[0],acc_vector[1],acc_vector[2]), "press "r" to exit press space to save to yaml")
        
    def yaml_file_dump(self):
        name_list = ['gyro_offset','acc_offset','acc_scaling']
        config_list = [self.gyro_correction_list, self.acc_correction_offset_list,self.acc_correction_scaling_list]
        output = []
        output_simple = []
        for name, config in zip(name_list, config_list):
            output_simple.append({name:config})
        with open(self.absolute_path+self.imu_name+"_sensor_config.yaml", 'w+') as f:
            f.write('# autogenerated: Do not edit #\n')
            for entry in output_simple:
                f.write(yaml.dump(entry))
        raw_input("press any key to load")
        x = yaml.load(file(self.absolute_path+self.imu_name+"_sensor_config.yaml"))
        raw_input("press any key to print")
        print (x)
        
    def write_sub(self, vector, parameter):
        output = Vector3Stamped()
        output.header.frame_id = parameter
        output.vector.x = vector[0]
        output.vector.y = vector[1]
        output.vector.z = vector[2]
        return output



if __name__=='__main__':
    rospy.init_node("imu_sensors_calib_node")
    calib_object = imu_sensor_calibration()
    gyro_calib_status = calib_object.calibrate_gyros()
    print "Gyro calibration status: "+str(gyro_calib_status)
    acc_x_calib_status = calib_object.calibrate_accelerometer("X axis", 1, 0)
    acc_x_calib_status = calib_object.calibrate_accelerometer("X axis", -1, 0)
    acc_y_calib_status = calib_object.calibrate_accelerometer("Y axis", 1, 1)
    acc_y_calib_status = calib_object.calibrate_accelerometer("Y axis", -1, 1)
    acc_z_calib_status = calib_object.calibrate_accelerometer("Z axis", 1, 2)
    acc_z_calib_status = calib_object.calibrate_accelerometer("Z axis", -1, 2)
    raw_input("press any key to enter test mode")
    calib_object.calib_check()
    print("Please exit the program")
    rospy.spin()