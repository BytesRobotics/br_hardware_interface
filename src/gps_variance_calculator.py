#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import numpy as np
import tf

covariance_matrix = np.zeros((6, 6))
max_samples = 5000
samples = np.zeros((max_samples, 6))
num_samples = 0


def compute_matrix(average_covariance=False):
    global num_samples, max_samples, covariance_matrix, samples
    old_covariance_matrix = covariance_matrix
    mean = np.sum(samples, axis=1) / num_samples
    print("mean: " + str(mean))
    for sample in samples:
        for i in range(6):
            covariance_matrix[i, i] += (mean[i] - sample[i]) ** 2
    covariance_matrix /= num_samples
    print(covariance_matrix)
    if average_covariance:
        covariance_matrix = old_covariance_matrix + covariance_matrix


def new_odom(odom):
    global num_samples, max_samples, covariance_matrix, samples

    if num_samples < max_samples:  # collect max_samples number of test points for calculation
        samples[num_samples, 0] = odom.twist.twist.linear.x
        samples[num_samples, 1] = odom.twist.twist.linear.y
        samples[num_samples, 2] = odom.twist.twist.linear.z
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)
        orientation = tf.transformations.euler_from_quaternion(quaternion)
        samples[num_samples, 3] = orientation[0]
        samples[num_samples, 4] = orientation[1]
        samples[num_samples, 5] = orientation[2]
        num_samples += 1


# Position covariance [m^2] defined relative to a tangential plane
# through the reported position. The components are East, North, and
# Up (ENU), in row-major order.
def new_fix(navSatFix):
    global num_samples, max_samples, covariance_matrix, samples

    if num_samples < max_samples:  # collect max_samples number of test points for calculation
        samples[num_samples, 0] = navSatFix.latitude
        samples[num_samples, 1] = navSatFix.longitude
        samples[num_samples, 2] = navSatFix.altitude
        samples[num_samples, 3] = 0
        samples[num_samples, 4] = 0
        samples[num_samples, 5] = 0
        num_samples += 1


if __name__ == '__main__':
    rospy.init_node("covariance_calculator")

    global max_samples, num_samples
    raw_input("start?")
    print("starting data collection, keep the sensor/tracked object still")
    # rospy.Subscriber("/odom", Odometry, new_odom)  # for updating state with odometry
    rospy.Subscriber("/gps", NavSatFix, new_fix)  # for updating state with gps
    while num_samples < max_samples:
        pass
    compute_matrix(False)
    continue_response = raw_input(
        "Would you like to continue and average covariance matrix from multiple points? y/n").lower()
    cnt = 1
    while continue_response == "y":
        cnt += 1
        num_samples = 0
        while num_samples < max_samples:
            pass
        compute_matrix(True)
        continue_response = raw_input(
            "Would you like to continue and average covariance matrix from multiple points? y/n").lower()
    print("cnt: " + str(cnt))
    print(covariance_matrix/cnt)