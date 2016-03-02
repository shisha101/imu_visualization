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
        self.number_of_points = len(self.data_dict[self.dict_entry][0])
        self.ones_vector = np.ones(self.number_of_points)

    def create_reading_matrix_x_sqr(self):
        np_mag_values = np.array(self.data_dict[self.dict_entry])
        # np_mag_values = self.normalize_readings()
        squared_values = np_mag_values * np_mag_values  # element wise multiplication correct
        measurement_matrix = np.vstack((np_mag_values, -1.0 * squared_values[1:3], self.ones_vector))
        measurement_matrix = np.matrix(measurement_matrix.T)
        # print self.data_dict[self.dict_entry][0][0:5]
        # print self.data_dict[self.dict_entry][1][0:5]
        # print self.data_dict[self.dict_entry][2][0:5]
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
        print "the number of points used is: %i" % self.number_of_points
        print "the condition number of the measurement matrix is %f " % np.linalg.cond(measurement_matrix)
        # print "Hi"
        # print pinv.shape
        # print np.matrix(self.ones_vector).shape
        # print np.linalg.cond(pinv)
        # parameters = pinv * np.matrix(sq_vals_np[0]).T
        parameters, residual, p, pp = np.linalg.lstsq(measurement_matrix,  np.matrix(sq_vals_np[0]).T)
        parameters = np.squeeze(np.asarray(parameters))  # convert from matrix to array
        # print parameters

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
        A = parameters[5] + bias[0]**2 + parameters[3] * bias[1]**2 + parameters[4] * bias[2]**2
        B = A / parameters[3]
        C = A / parameters[4]
        sensitivity.append(np.sqrt(1/A))
        sensitivity.append(np.sqrt(1/B))
        sensitivity.append(np.sqrt(1/C))
        self.calib_values["sensitivities"] = sensitivity
        print "the scaling factors for the magnetometer are x: %s, y: %s, z: %s" % (self.calib_values["sensitivities"][0],
                                                                               self.calib_values["sensitivities"][1],
                                                                               self.calib_values["sensitivities"][2])
        print "the biases for the magnetometer are x: %s, y: %s, z: %s" % (self.calib_values["bias"][0],
                                                                      self.calib_values["bias"][1],
                                                                      self.calib_values["bias"][2])
        print "the quality without calibration is %s positive and negative percent off > |4| is bad" % (max(abs(np.array(bias))) * max(sensitivity))
        print "the residual is %s" % (residual[0, 0]/(self.number_of_points))
        self.debug_magnitude(measurement_matrix, bias, sensitivity)

    def debug_magnitude(self, measurement_matrix, bias, sensitivity):
        b = np.array(bias)
        sens_array = np.array(sensitivity)
        bias_corrected_measurements = measurement_matrix[:, 0:3] - b
        corrected_values = np.multiply(bias_corrected_measurements, sens_array)
        magnitude_sq = np.multiply(corrected_values, corrected_values).sum(axis=1)
        magnitude = np.sqrt(magnitude_sq)
        print magnitude
        print "the average magnitude after projection is: %f " % np.average(magnitude)
        print "debug"
        print "bias"
        print b
        print "sensitivity"
        print sens_array
        print "raw data"
        print measurement_matrix[0:3, 0:3]
        print "bias corrected"
        print bias_corrected_measurements[0:3, :]
        print "corrected"
        print corrected_values[0:3, :]
        print "mag_sq"
        print magnitude_sq[0:3, :]
        print "magnitude"
        print magnitude[0:3]

    def create_reading_matrix_hard_iron(self):
        np_mag_values = np.array(self.data_dict[self.dict_entry])
        squared_values = np_mag_values * np_mag_values  # element wise multiplication correct
        measurement_matrix = np.vstack((np_mag_values, self.ones_vector))
        measurement_matrix = np.matrix(measurement_matrix.T)
        return measurement_matrix, squared_values

    def calculate_calib_parameters_hard_iron(self):
        measurement_matrix, sq_vals_np = self.create_reading_matrix_hard_iron()
        print "the number of points used is: %i" % self.number_of_points
        print "the condition number of the measurement matrix is %f " % np.linalg.cond(measurement_matrix)
        pinv = np.linalg.pinv(measurement_matrix)
        # print np.matrix(sq_vals_np.sum(axis=0)).T.shape
        # parameters = pinv * np.matrix(sq_vals_np.sum(axis=0)).T
        parameters, residual, p, pp = np.linalg.lstsq(measurement_matrix,  np.matrix(sq_vals_np.sum(axis=0)).T)
        parameters = np.squeeze(np.asarray(parameters))  # convert from matrix to array
        bias = (parameters[0:3]/2.0)
        self.calib_values["bias"] = bias
        print "the scaling factors for the magnetometer are x: %s, y: %s, z: %s" % (1.0, 1.0, 1.0)
        print "the biases for the magnetometer are x: %s, y: %s, z: %s" % (self.calib_values["bias"][0],
                                                                      self.calib_values["bias"][1],
                                                                      self.calib_values["bias"][2])
        magnetometer_vector_magnitude = np.sqrt(np.square(bias).sum() + parameters[3])
        print "the magnitude vector of the magnetometer is :%s" % magnetometer_vector_magnitude
        print "the quality without calibration is %s percent off > 4 is bad" % ((abs(bias).max()/magnetometer_vector_magnitude)*100)
        print "the residual is %s" % (residual[0, 0]/(self.number_of_points))

    def check(self):
        pass

    def dump_to_file(self):
        pass


if __name__ == '__main__':
    file_name_of_data_default = "Xsens_data_onboard_office.npy"
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
    # imu_mag_calib_obj.calculate_calib_parameters_norm()

