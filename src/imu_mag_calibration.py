import numpy as np
import sys
from itertools import islice

class imu_mag_calibration(object):
    def __init__(self, data_dict):
        """
        :param data_dict: a dictionary containing the imu data, must contain mag data see capture_imu_data.py
        """
        self.dict_entry = "mag"
        self.data_dict = data_dict
        self.calib_values = {"sensitivities": None, "bias": None}
        self.ones_vector = np.ones(len(self.data_dict[self.dict_entry][0]))

    def create_reading_matrix(self):
        np_mag_values = np.array(self.data_dict[self.dict_entry])
        # np_mag_values = self.normalize_readings()
        squared_values = np_mag_values * np_mag_values  # element wise multiplication correct
        measurement_matrix = np.vstack((np_mag_values, squared_values, self.ones_vector))
        measurement_matrix = np.matrix(measurement_matrix.T)
        # print self.data_dict[self.dict_entry][0][0:5]
        # print self.data_dict[self.dict_entry][1][0:5]
        # print self.data_dict[self.dict_entry][2][0:5]
        # print squared_values[0][0:5]
        # print squared_values[1][0:5]
        # print squared_values[2][0:5]
        print measurement_matrix[0:7]
        print np.linalg.cond(measurement_matrix.T * measurement_matrix)
        print np.linalg.cond(measurement_matrix)
        print np.linalg.matrix_rank(measurement_matrix)
        print "********"
        return measurement_matrix

    def calculate_calib_parameters_norm(self):
        measurement_matrix = self.create_reading_matrix()
        pinv = np.linalg.pinv(measurement_matrix)
        print "Hi"
        print pinv.shape
        print np.matrix(self.ones_vector).shape
        print np.linalg.cond(pinv)
        parameters = pinv * np.matrix(self.ones_vector).T
        print parameters
        # parameters = np.linalg.lstsq(measurement_matrix, self.ones_vector)
        print "the parameters"
        print parameters
        # print parameters_1
        self.calib_values["sensitivities"] = np.sqrt(parameters[3:6])
        print type(self.calib_values["sensitivities"])
        print self.calib_values["sensitivities"].shape
        print self.calib_values["sensitivities"]
        print np.sqrt(parameters[3:5])
        self.calib_values["bias"] = [-1/(2*parameters[0]), -1/(2*parameters[1]), -1/(2*parameters[2])]
        print "the scaling factors for the magnetometer are x:%s, y%s, z%s" % (self.calib_values["sensitivities"][0],
                                                                               self.calib_values["sensitivities"][1],
                                                                               self.calib_values["sensitivities"][2])
        print "the biases for the magnetometer are x:%s, y%s, z%s" % (self.calib_values["bias"][0],
                                                                      self.calib_values["bias"][1],
                                                                      self.calib_values["bias"][2])

    def create_reading_matrix_x_sqr(self):
        np_mag_values = np.array(self.data_dict[self.dict_entry])
        # np_mag_values = self.normalize_readings()
        squared_values = np_mag_values * np_mag_values  # element wise multiplication correct
        measurement_matrix = np.vstack((np_mag_values, -1.0 * squared_values[1:3], self.ones_vector))
        measurement_matrix = np.matrix(measurement_matrix.T)
        print self.data_dict[self.dict_entry][0][0:5]
        print self.data_dict[self.dict_entry][1][0:5]
        print self.data_dict[self.dict_entry][2][0:5]
        # print squared_values[0][0:5]
        # print squared_values[1][0:5]
        # print squared_values[2][0:5]
        # print measurement_matrix[0:7]
        # print measurement_matrix.shape
        # print np.linalg.cond(measurement_matrix.T * measurement_matrix)
        # print np.linalg.cond(measurement_matrix)
        # print np.linalg.matrix_rank(measurement_matrix)
        print "********"
        return measurement_matrix, squared_values

    def calculate_calib_parameters_x_sqr(self):
        measurement_matrix, sq_vals_np = self.create_reading_matrix_x_sqr()
        pinv = np.linalg.pinv(measurement_matrix)
        # print "Hi"
        # print pinv.shape
        # print np.matrix(self.ones_vector).shape
        # print np.linalg.cond(pinv)
        parameters = pinv * np.matrix(sq_vals_np[0]).T
        # print parameters
        # parameters = np.linalg.lstsq(measurement_matrix,  np.matrix(sq_vals_np[0]).T)
        # print "the parameters"
        # print parameters
        # print parameters_1
        # self.calib_values["sensitivities"] = np.sqrt(parameters[3:6])
        # print type(self.calib_values["sensitivities"])
        # print self.calib_values["sensitivities"].shape
        # print self.calib_values["sensitivities"]
        # print np.sqrt(parameters[3:5])
        bias = []
        bias.append(parameters[0]/2)
        bias.append(parameters[1]/(2*parameters[3]))
        bias.append(parameters[2]/(2*parameters[4]))
        self.calib_values["bias"] = bias
        sensitivity = []
        A = parameters[5] + bias[0] + parameters[3] * bias[1] + parameters[4] * bias[2]
        B = A / parameters[3]
        C = A / parameters[4]
        sensitivity.append(np.sqrt(1/A))
        sensitivity.append(np.sqrt(1/B))
        sensitivity.append(np.sqrt(1/C))
        self.calib_values["sensitivities"] = sensitivity
        print "the scaling factors for the magnetometer are x:%s, y%s, z%s" % (self.calib_values["sensitivities"][0],
                                                                               self.calib_values["sensitivities"][1],
                                                                               self.calib_values["sensitivities"][2])
        print "the biases for the magnetometer are x:%s, y%s, z%s" % (self.calib_values["bias"][0],
                                                                      self.calib_values["bias"][1],
                                                                      self.calib_values["bias"][2])
        print "the quality without calibration is %s positive and %s negative percent off > |4| is bad" % (max(bias)*100, min(bias)*100)
        print (measurement_matrix[0, 0:4] - np.array([bias]))

    def create_reading_matrix_hard_iron(self):
        np_mag_values = np.array(self.data_dict[self.dict_entry])
        squared_values = np_mag_values * np_mag_values  # element wise multiplication correct
        measurement_matrix = np.vstack((np_mag_values, self.ones_vector))
        measurement_matrix = np.matrix(measurement_matrix.T)
        print np.linalg.cond(measurement_matrix)
        return measurement_matrix, squared_values

    def calculate_calib_parameters_hard_iron(self):
        measurement_matrix, sq_vals_np = self.create_reading_matrix_hard_iron()
        pinv = np.linalg.pinv(measurement_matrix)
        print np.matrix(sq_vals_np.sum(axis=0)).T.shape
        parameters = pinv * np.matrix(sq_vals_np.sum(axis=0)).T
        bias = (parameters[0:3]/2.0)
        self.calib_values["bias"] = bias
        print "the scaling factors for the magnetometer are x: %s, y: %s, z: %s" % (1.0, 1.0, 1.0)
        print "the biases for the magnetometer are x:%s, y%s, z%s" % (self.calib_values["bias"][0],
                                                                      self.calib_values["bias"][1],
                                                                      self.calib_values["bias"][2])
        magnetometer_vector_magnitude = np.sqrt(np.square(bias).sum() + parameters[3])
        print "the magnitude vector of the magnetometer is :%s" % magnetometer_vector_magnitude
        print "the quality without calibration is %s percent off > 4 is bad" % ((abs(bias).max()/magnetometer_vector_magnitude)*100)

    def normalize_readings(self):
        np_array = np.array([self.data_dict["mag"][0], self.data_dict["mag"][1], self.data_dict["mag"][2]])
        # np*np element wise mul sum(axis=0) sum all squared components
        magnitude = np.sqrt((np_array*np_array).sum(axis=0))
        np_array = np_array/magnitude
        return np_array

    def check(self):
        pass

    def residual(self):
        pass

    def dump_to_file(self):
        pass


if __name__ == '__main__':
    file_name_of_data_default = "xIMU_data_onboard_office.npy"
    path_of_file_load_default = "/home/abs8rng/catkin_ws/src/my_packages/low_cost_navigation/evaluation_code/data/imu_recordings/"
    if len(sys.argv) < 2:
        print "no file specified. Using default value"
        file_name_of_data = file_name_of_data_default
        path_of_file_load = path_of_file_load_default
    else:
        file_name_of_data = sys.argv[1]
        if len(sys.argv) == 3:
            path_of_file_load = sys.argv[2]
        else:
            path_of_file_load = path_of_file_load_default

    dic_from_drive = np.load(path_of_file_load + file_name_of_data).item()
    imu_mag_calib_obj = imu_mag_calibration(dic_from_drive)
    imu_mag_calib_obj.calculate_calib_parameters_hard_iron()
    imu_mag_calib_obj.calculate_calib_parameters_x_sqr()

