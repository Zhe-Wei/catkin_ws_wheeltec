#!/usr/bin/env python3

# Import necessary libraries
import rospy
import tf
from tf import transformations
from tf import broadcaster
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math
import numpy as np
from matplotlib import pyplot as plt

import skfuzzy as fuzz
from skfuzzy import control as ctrl
import skfuzzy.control.visualization as viz

# show plots
SHOW_PLOTS = False
PUBLISH_VEL = True
DEBUG = False
ANG_VEL_INFER = True
# Master robot's velocity
odom_linear_x=0
odom_linear_y=0
odom_angular_z=0

# Slave robot's velocity
slave_linear_x=0
slave_linear_y=0
slave_angular_z=0

# Error
e_angular_x = 0
e_angular_y = 0
e_angular_z = 0

e_linear_x = 0
e_linear_y = 0
e_linear_z = 0


# Define a function to continuously publish velocity commands


def move():

    ### Calculate the angular turn to the slave
    if math.fabs(e_linear_x) < 5:  # x軸差距小於5時，以leader的r計算
        angular_turn = math.atan2(slave_y, odom_linear_x/odom_angular_z)
    else:  # x軸差距大於5時
        angular_turn = math.atan2(slave_y, slave_x)

    def angular_vel_inference():
        ### Input range
        max_angular_error = 3.14 # angular_error 範圍
        max_angular_vel = 0.3 # angular_vel 範圍
        angular_error = np.arange(-3.14, 3.14, 0.001)
        angular_vel = np.arange(-0.3, 0.3, 0.001)

        ### Output range
        max_acc_angular = 1 # acc_angular 範圍
        acc_angular = np.arange(-1, 1, 0.001)

        ### Generate Input fuzzy membership functions
        angular_error_very_neg = fuzz.trapmf(angular_error, [-3.14, -3.14, -2, -1.5])
        angular_error_neg = fuzz.trimf(angular_error, [-2, -1, 0])
        angular_error_zero = fuzz.trimf(angular_error, [-1, 0, 1])
        angular_error_pos = fuzz.trimf(angular_error, [0, 1, 2])
        angular_error_very_pos = fuzz.trapmf(angular_error, [1.5, 2, 3.14, 3.14])

        angular_vel_very_neg = fuzz.trapmf(angular_vel, [-0.3, -0.3, -0.2, -0.15])
        angular_vel_neg = fuzz.trimf(angular_vel, [-0.25, -0.125, 0])
        angular_vel_zero = fuzz.trimf(angular_vel, [-0.1, 0, 0.1])
        angular_vel_pos = fuzz.trimf(angular_vel, [0, 0.125, 0.25])
        angular_vel_very_pos = fuzz.trapmf(angular_vel, [0.15, 0.2, 0.3, 0.3])
   

        ### Generate Output fuzzy membership functions
        acc_angular_very_neg = fuzz.trapmf(acc_angular, [-1, -1, -0.5, -0.25])
        acc_angular_neg = fuzz.trimf(acc_angular, [-0.5, -0.25, 0])
        acc_angular_zero = fuzz.trimf(acc_angular, [-0.25, 0, 0.25])
        acc_angular_pos = fuzz.trimf(acc_angular, [0, 0.25, 0.5])
        acc_angular_very_pos = fuzz.trapmf(acc_angular, [0.25, 0.5, 1, 1])
        if DEBUG:
            print("acc_angular", acc_angular_zero)

        ### Input data for fuzzy inference and use stauration to limit the input value
        z_error_value = -e_angular_z
        z_error_value = min(max(z_error_value, -max_angular_error), max_angular_error)

        angular_vel_value = slave_angular_z
        angular_vel_value = min(max(angular_vel_value, -max_angular_vel), max_angular_vel)

        ### Activate fuzzy membership functions at the input value
        z_error_level_very_neg = fuzz.interp_membership(angular_error, angular_error_very_neg, z_error_value)
        z_error_level_neg = fuzz.interp_membership(angular_error, angular_error_neg, z_error_value)
        z_error_level_zero = fuzz.interp_membership(angular_error, angular_error_zero, z_error_value)
        z_error_level_pos = fuzz.interp_membership(angular_error, angular_error_pos, z_error_value)
        z_error_level_very_pos = fuzz.interp_membership(angular_error, angular_error_very_pos, z_error_value)
        if DEBUG:
            print("z_error_level")
            print(round(z_error_level_very_neg, 2), round(z_error_level_neg, 2), round(z_error_level_zero, 2), round(z_error_level_pos, 2), round(z_error_level_very_pos, 2))

        angular_vel_level_very_neg = fuzz.interp_membership(angular_vel, angular_vel_very_neg, angular_vel_value)
        angular_vel_level_neg = fuzz.interp_membership(angular_vel, angular_vel_neg, angular_vel_value)
        angular_vel_level_zero = fuzz.interp_membership(angular_vel, angular_vel_zero, angular_vel_value)
        angular_vel_level_pos = fuzz.interp_membership(angular_vel, angular_vel_pos, angular_vel_value)
        angular_vel_level_very_pos = fuzz.interp_membership(angular_vel, angular_vel_very_pos, angular_vel_value)
        if DEBUG:
            print("angular_vel_level")
            print(round(angular_vel_level_very_neg, 2), round(angular_vel_level_neg, 2), round(angular_vel_level_zero, 2), round(angular_vel_level_pos, 2), round(angular_vel_level_very_pos, 2))


        ### Visualize the membership functions and show degree on the plot
        if SHOW_PLOTS:
            fig, (ax00, ax01, ax02, ax03, ax04) = plt.subplots(nrows=5, figsize=(8, 9))
            z_error_0 = np.zeros_like(angular_error)
            
            # ax01.plot(x_error, x_error_very_neg, 'b', linewidth=1.5, label='Very Negative=' + str(round(x_error_level_very_neg, 2)))
            # ax01.fill_between(x_error, x_error_0, np.fmin(
            #     x_error_level_very_neg, x_error_very_neg), facecolor='b', alpha=0.7)
            ax00.plot(angular_error, angular_error_very_neg, 'b', linewidth=1.5, label='Very Negative=' + str(round(z_error_level_very_neg, 2)))
            ax00.fill_between(angular_error, z_error_0, np.fmin(z_error_level_very_neg, angular_error_very_neg), facecolor='b', alpha=0.7)
            ax00.plot(angular_error, angular_error_neg, 'g', linewidth=1.5, label='Negative=' + str(round(z_error_level_neg, 2)))
            ax00.fill_between(angular_error, z_error_0, np.fmin(z_error_level_neg, angular_error_neg), facecolor='g', alpha=0.7)
            ax00.plot(angular_error, angular_error_zero, 'r', linewidth=1.5, label='Zero=' + str(round(z_error_level_zero, 2)))
            ax00.fill_between(angular_error, z_error_0, np.fmin(z_error_level_zero, angular_error_zero), facecolor='r', alpha=0.7)
            ax00.plot(angular_error, angular_error_pos, 'c', linewidth=1.5, label='Positive=' + str(round(z_error_level_pos, 2)))
            ax00.fill_between(angular_error, z_error_0, np.fmin(z_error_level_pos, angular_error_pos), facecolor='c', alpha=0.7)
            ax00.plot(angular_error, angular_error_very_pos, 'm', linewidth=1.5, label='Very Positive=' + str(round(z_error_level_very_pos, 2)))
            ax00.fill_between(angular_error, z_error_0, np.fmin(z_error_level_very_pos, angular_error_very_pos), facecolor='m', alpha=0.7)
            ax00.set_title('Z Error= ' + str(round(z_error_value, 2)) + 'rad')
            ax00.legend()

            angular_vel_0 = np.zeros_like(angular_vel)
            ax01.plot(angular_vel, angular_vel_very_neg, 'b', linewidth=1.5, label='Very Negative=' + str(round(angular_vel_level_very_neg, 2)))
            ax01.fill_between(angular_vel, angular_vel_0, np.fmin(angular_vel_level_very_neg, angular_vel_very_neg), facecolor='b', alpha=0.7)
            ax01.plot(angular_vel, angular_vel_neg, 'g', linewidth=1.5, label='Negative=' + str(round(angular_vel_level_neg, 2)))
            ax01.fill_between(angular_vel, angular_vel_0, np.fmin(angular_vel_level_neg, angular_vel_neg), facecolor='g', alpha=0.7)
            ax01.plot(angular_vel, angular_vel_zero, 'r', linewidth=1.5, label='Zero=' + str(round(angular_vel_level_zero, 2)))
            ax01.fill_between(angular_vel, angular_vel_0, np.fmin(angular_vel_level_zero, angular_vel_zero), facecolor='r', alpha=0.7)
            ax01.plot(angular_vel, angular_vel_pos, 'c', linewidth=1.5, label='Positive=' + str(round(angular_vel_level_pos, 2)))
            ax01.fill_between(angular_vel, angular_vel_0, np.fmin(angular_vel_level_pos, angular_vel_pos), facecolor='c', alpha=0.7)
            ax01.plot(angular_vel, angular_vel_very_pos, 'm', linewidth=1.5, label='Very Positive=' + str(round(angular_vel_level_very_pos, 2)))
            ax01.fill_between(angular_vel, angular_vel_0, np.fmin(angular_vel_level_very_pos, angular_vel_very_pos), facecolor='m', alpha=0.7)
            ax01.set_title('Angular Velocity= ' + str(round(angular_vel_value, 2)) + 'rad/s')
            ax01.legend()

            ax02.plot(acc_angular, acc_angular_very_neg, 'b', linewidth=0.5, linestyle='--', label='Very Negative')
            ax02.plot(acc_angular, acc_angular_neg, 'g', linewidth=0.5, linestyle='--', label='Negative')
            ax02.plot(acc_angular, acc_angular_zero, 'r', linewidth=0.5, linestyle='--', label='Zero')
            ax02.plot(acc_angular, acc_angular_pos, 'c', linewidth=0.5, linestyle='--', label='Positive')
            ax02.plot(acc_angular, acc_angular_very_pos, 'm', linewidth=0.5, linestyle='--', label='Very Positive')

            ax02.set_title('Acceration Angular Velocity')
            ax02.legend()

        ### Rules
        acc_z_very_neg_rules = [
            np.fmin(z_error_level_zero, angular_vel_level_very_pos),
            np.fmin(z_error_level_pos, angular_vel_level_pos),
            np.fmin(z_error_level_pos, angular_vel_level_very_pos),
            np.fmin(z_error_level_very_pos, angular_vel_level_zero),
            np.fmin(z_error_level_very_pos, angular_vel_level_pos),
            np.fmin(z_error_level_very_pos, angular_vel_level_very_pos)
        ]

        acc_z_neg_rules = [
            np.fmin(z_error_level_neg, angular_vel_level_very_pos),
            np.fmin(z_error_level_zero, angular_vel_level_pos),
            np.fmin(z_error_level_pos, angular_vel_level_zero),
            np.fmin(z_error_level_very_pos, angular_vel_level_very_neg),
            np.fmin(z_error_level_very_pos, angular_vel_level_neg),
        ]

        acc_z_zero_rules = [
            np.fmin(z_error_level_neg, angular_vel_level_pos),
            np.fmin(z_error_level_zero, angular_vel_level_zero),
            np.fmin(z_error_level_pos, angular_vel_level_neg),

        ]

        acc_z_pos_rules = [
            np.fmin(z_error_level_very_neg, angular_vel_level_pos),
            np.fmin(z_error_level_very_neg, angular_vel_level_very_pos),
            np.fmin(z_error_level_neg, angular_vel_level_zero),
            np.fmin(z_error_level_zero, angular_vel_level_neg),
            np.fmin(z_error_level_pos, angular_vel_level_very_neg),
        ]

        acc_z_very_pos_rules = [
            np.fmin(z_error_level_very_neg, angular_vel_level_very_neg),
            np.fmin(z_error_level_very_neg, angular_vel_level_neg),
            np.fmin(z_error_level_very_neg, angular_vel_level_zero),
            np.fmin(z_error_level_neg, angular_vel_level_very_neg),
            np.fmin(z_error_level_neg, angular_vel_level_neg),
            np.fmin(z_error_level_zero, angular_vel_level_very_neg),
        ]
        if DEBUG:
            print("####rules#### ")
            print('acc_z_very_neg_rules', acc_z_very_neg_rules)
            print('acc_z_neg_rules', acc_z_neg_rules)
            print('acc_z_zero_rules', acc_z_zero_rules)
            print('acc_z_pos_rules', acc_z_pos_rules)
            print('acc_z_very_pos_rules', acc_z_very_pos_rules)

        avg_acc_z_very_neg_rules = np.mean(list(filter(lambda x: x.size > 0, acc_z_very_neg_rules)))
        avg_acc_z_neg_rules = np.mean(list(filter(lambda x: x.size > 0, acc_z_neg_rules)))
        avg_acc_z_zero_rules = np.mean(list(filter(lambda x: x.size > 0, acc_z_zero_rules)))
        avg_acc_z_pos_rules = np.mean(list(filter(lambda x: x.size > 0, acc_z_pos_rules)))
        avg_acc_z_very_pos_rules = np.mean(list(filter(lambda x: x.size > 0, acc_z_very_pos_rules)))
        if DEBUG:
            print("####avg_rules#### ")
            print('avg_acc_z_very_neg_rules', avg_acc_z_very_neg_rules)
            print('avg_acc_z_neg_rules', avg_acc_z_neg_rules)
            print('avg_acc_z_zero_rules', avg_acc_z_zero_rules)
            print('avg_acc_z_pos_rules', avg_acc_z_pos_rules)
            print('avg_acc_z_very_pos_rules', avg_acc_z_very_pos_rules)

        # Rule aggregation
        if DEBUG:
            print("#### aggregation ####")
        agg_acc_z_very_neg = np.fmin(avg_acc_z_very_neg_rules, acc_angular_very_neg)
        agg_acc_z_neg = np.fmin(avg_acc_z_neg_rules, acc_angular_neg)
        agg_acc_z_zero = np.fmin(avg_acc_z_zero_rules, acc_angular_zero)
        agg_acc_z_pos = np.fmin(avg_acc_z_pos_rules, acc_angular_pos)
        agg_acc_z_very_pos = np.fmin(avg_acc_z_very_pos_rules, acc_angular_very_pos)

        if DEBUG:
            print("agg_acc_z_very_neg", agg_acc_z_very_neg)
            print("agg_acc_z_neg", agg_acc_z_neg)
            print("agg_acc_z_zero", agg_acc_z_zero)
            print("agg_acc_z_pos", agg_acc_z_pos)
            print("agg_acc_z_very_pos", agg_acc_z_very_pos)


        acc_z_0 = np.zeros_like(acc_angular)

        # print("#### shape ####")
        # print("acc_angular shape", acc_angular.shape)
        # print("acc_z_very_neg", acc_z_very_neg)
        # print("acc_z_neg", acc_z_neg)
        # print("acc_z_zero", acc_z_zero)
        # print("acc_z_pos", acc_z_pos)
        # print("acc_z_very_pos", acc_z_very_pos)



        # Visualize 
        if SHOW_PLOTS:
            ax03.fill_between(acc_angular, acc_z_0, agg_acc_z_very_neg, facecolor='b', alpha=0.7)
            ax03.plot(acc_angular, acc_angular_very_neg, 'b', linewidth=0.5, linestyle='--', )
            ax03.fill_between(acc_angular, acc_z_0, agg_acc_z_neg, facecolor='g', alpha=0.7)
            ax03.plot(acc_angular, acc_angular_neg, 'g', linewidth=0.5, linestyle='--')
            ax03.fill_between(acc_angular, acc_z_0, agg_acc_z_zero, facecolor='r', alpha=0.7)
            ax03.plot(acc_angular, acc_angular_zero, 'r', linewidth=0.5, linestyle='--')
            ax03.fill_between(acc_angular, acc_z_0, agg_acc_z_pos, facecolor='y', alpha=0.7)
            ax03.plot(acc_angular, acc_angular_pos, 'y', linewidth=0.5, linestyle='--')
            ax03.fill_between(acc_angular, acc_z_0, agg_acc_z_very_pos, facecolor='m', alpha=0.7)
            ax03.plot(acc_angular, acc_angular_very_pos, 'm', linewidth=0.5, linestyle='--')
            ax03.set_title('Output membership activity')
        
        # Defuzzification
        agg = np.fmax(agg_acc_z_very_neg, np.fmax(agg_acc_z_neg, np.fmax(agg_acc_z_zero, np.fmax(agg_acc_z_pos, agg_acc_z_very_pos))))

        try:
            centoid_acc_z = fuzz.defuzz(acc_angular, agg, 'centroid')
        except:
            centoid_acc_z = 0

        if DEBUG:
            print("centoid_acc_z", centoid_acc_z, slave_angular_z+centoid_acc_z)

        defuzz_acc_z = fuzz.interp_membership(acc_angular, agg, centoid_acc_z)  # for plot

        if SHOW_PLOTS:
            # Visualize
            ax04.plot(acc_angular, acc_angular_very_neg, 'b', linewidth=0.5, linestyle='--', )
            ax04.plot(acc_angular, acc_angular_neg, 'g', linewidth=0.5, linestyle='--')
            ax04.plot(acc_angular, acc_angular_zero, 'r', linewidth=0.5, linestyle='--')
            ax04.plot(acc_angular, acc_angular_pos, 'y', linewidth=0.5, linestyle='--')
            ax04.plot(acc_angular, acc_angular_very_pos, 'm', linewidth=0.5, linestyle='--')
            ax04.fill_between(acc_angular, acc_z_0, agg, facecolor='Orange', alpha=0.7)
            ax04.plot([centoid_acc_z, centoid_acc_z], [0, defuzz_acc_z], 'k', linewidth=1.5, alpha=0.9)
            ax04.set_title('Aggregated membership and result (line)')
            ax04.text(centoid_acc_z, 0.5, round(centoid_acc_z, 2), color='k', fontsize=10, ha='center', va='center')

            for ax in (ax00, ax01, ax02, ax03, ax04):
                ax.spines['top'].set_visible(False)
                ax.spines['right'].set_visible(False)
                ax.get_xaxis().set_ticks([])
                ax.get_yaxis().set_ticks([])


        ### Generate fuzzy system

        return centoid_acc_z

    def linear_vel_inference():
        ### Input range
        max_x_error = 2 # x_error 範圍
        x_error = np.arange(-2, 2, 0.01)
        linear_vel = np.arange(-2, 2, 0.01)

        ### Output range
        acc_x = np.arange(-1, 1, 0.1)

        ### Generate Input fuzzy membership functions
        x_error_very_neg = fuzz.trapmf(x_error, [-2, -2, -1.625, -0.75])
        x_error_neg = fuzz.trimf(x_error, [-1, -0.5625, -0.125])
        x_error_zero = fuzz.trimf(x_error, [-0.25, 0, 0.25])
        x_error_pos = fuzz.trimf(x_error, [0.125, 0.5625, 1])
        x_error_very_pos = fuzz.trapmf(x_error, [0.75, 1.625, 2, 2])

        linear_vel_very_neg = fuzz.trapmf(linear_vel, [-2, -2, -1.625, -0.75])
        linear_vel_neg = fuzz.trimf(linear_vel, [-1.625, -0.75, 0])
        linear_vel_zero = fuzz.trimf(linear_vel, [-0.75, 0, 0.75])
        linear_vel_pos = fuzz.trimf(linear_vel, [0, 0.75, 1.625])
        linear_vel_very_pos = fuzz.trapmf(linear_vel, [0.75, 1.625, 2, 2])

        ### Generate Output fuzzy membership functions
        acc_x_very_neg = fuzz.trapmf(acc_x, [-1, -1, -0.8, -0.4])
        acc_x_neg = fuzz.trimf(acc_x, [-0.8, -0.4, 0])
        acc_x_zero = fuzz.trimf(acc_x, [-0.2, 0, 0.2])
        acc_x_pos = fuzz.trimf(acc_x, [0, 0.4, 0.8])
        acc_x_very_pos = fuzz.trapmf(acc_x, [0.4, 0.8, 1, 1])

        ### Input data for fuzzy inference and use stauration to limit the input value
        x_error_value = e_linear_x
        x_error_value = min(max(x_error_value, -max_x_error), max_x_error)  # 進行飽和運算 Ex:限制在 -2 ~ 2

        linear_vel_value = slave_linear_x
        linear_vel_value = min(max(linear_vel_value, -max_x_error), max_x_error)  # 進行飽和運算 Ex:限制在 -2 ~ 2


        ### Activation of fuzzy membership functions
        x_error_level_very_neg = fuzz.interp_membership(x_error, x_error_very_neg, x_error_value)
        x_error_level_neg = fuzz.interp_membership(x_error, x_error_neg, x_error_value)
        x_error_level_zero = fuzz.interp_membership(x_error, x_error_zero, x_error_value)
        x_error_level_pos = fuzz.interp_membership(x_error, x_error_pos, x_error_value)
        x_error_level_very_pos = fuzz.interp_membership(x_error, x_error_very_pos, x_error_value)
        if DEBUG:
            print("x_error_level")
            print(round(x_error_level_very_neg, 2), round(x_error_level_neg, 2), round(x_error_level_zero, 2), round(x_error_level_pos, 2), round(x_error_level_very_pos, 2))

        linear_vel_level_very_neg = fuzz.interp_membership(linear_vel, linear_vel_very_neg, linear_vel_value)
        linear_vel_level_neg = fuzz.interp_membership(linear_vel, linear_vel_neg, linear_vel_value)
        linear_vel_level_zero = fuzz.interp_membership(linear_vel, linear_vel_zero, linear_vel_value)
        linear_vel_level_pos = fuzz.interp_membership(linear_vel, linear_vel_pos, linear_vel_value)
        linear_vel_level_very_pos = fuzz.interp_membership(linear_vel, linear_vel_very_pos, linear_vel_value)
        if DEBUG:
            print("linear_vel_level")
            print(round(linear_vel_level_very_neg, 2), round(linear_vel_level_neg, 2), round(linear_vel_level_zero, 2), round(linear_vel_level_pos, 2), round(linear_vel_level_very_pos, 2))

        # Visualize the membership functions and show degrees(linear_vel_level) on the plot
        if SHOW_PLOTS:
            fig, (ax0, ax1, ax2, ax3, ax4) = plt.subplots(nrows=5, figsize=(8, 9))
            fig.subplots_adjust(hspace=0.5)
            x_error_0 = np.zeros_like(x_error)
            ax0.plot(x_error, x_error_very_neg, 'b', linewidth=1.5, label='Very Negative=' + str(round(x_error_level_very_neg, 2)))
            ax0.fill_between(x_error, x_error_0, np.fmin(x_error_level_very_neg, x_error_very_neg), facecolor='b', alpha=0.7)
            ax0.plot(x_error, x_error_neg, 'g', linewidth=1.5, label='Negative=' + str(round(x_error_level_neg, 2)))
            ax0.fill_between(x_error, x_error_0, np.fmin(x_error_level_neg, x_error_neg), facecolor='g', alpha=0.7)
            ax0.plot(x_error, x_error_zero, 'r', linewidth=1.5, label='Zero=' + str(round(x_error_level_zero, 2)))
            ax0.fill_between(x_error, x_error_0, np.fmin(x_error_level_zero, x_error_zero), facecolor='r', alpha=0.7)
            ax0.plot(x_error, x_error_pos, 'c', linewidth=1.5, label='Positive=' + str(round(x_error_level_pos, 2)))
            ax0.fill_between(x_error, x_error_0, np.fmin(x_error_level_pos, x_error_pos), facecolor='c', alpha=0.7)
            ax0.plot(x_error, x_error_very_pos, 'm', linewidth=1.5, label='Very Positive=' + str(round(x_error_level_very_pos, 2)))
            ax0.fill_between(x_error, x_error_0, np.fmin(x_error_level_very_pos, x_error_very_pos), facecolor='m', alpha=0.7)
            ax0.plot()
            ax0.set_title('Error in X=' + str(round(x_error_value, 2)))
            ax0.legend()

            linear_vel_0 = np.zeros_like(linear_vel)
            ax1.plot(linear_vel, linear_vel_very_neg, 'b', linewidth=1.5, label='Very Negative=' + str(round(linear_vel_level_very_neg, 2)))
            ax1.fill_between(linear_vel, linear_vel_0, np.fmin(linear_vel_level_very_neg, linear_vel_very_neg), facecolor='b', alpha=0.7)
            ax1.plot(linear_vel, linear_vel_neg, 'g', linewidth=1.5, label='Negative=' + str(round(linear_vel_level_neg, 2)))
            ax1.fill_between(linear_vel, linear_vel_0, np.fmin(linear_vel_level_neg, linear_vel_neg), facecolor='g', alpha=0.7)
            ax1.plot(linear_vel, linear_vel_zero, 'r', linewidth=1.5, label='Zero=' + str(round(linear_vel_level_zero, 2)))
            ax1.fill_between(linear_vel, linear_vel_0, np.fmin(linear_vel_level_zero, linear_vel_zero), facecolor='r', alpha=0.7)
            ax1.plot(linear_vel, linear_vel_pos, 'c', linewidth=1.5, label='Positive=' + str(round(linear_vel_level_pos, 2)))
            ax1.fill_between(linear_vel, linear_vel_0, np.fmin(linear_vel_level_pos, linear_vel_pos), facecolor='c', alpha=0.7)
            ax1.plot(linear_vel, linear_vel_very_pos, 'm', linewidth=1.5, label='Very Positive=' + str(round(linear_vel_level_very_pos, 2)))
            ax1.fill_between(linear_vel, linear_vel_0, np.fmin(linear_vel_level_very_pos, linear_vel_very_pos), facecolor='m', alpha=0.7)
            ax1.set_title('Linear Velocity=' + str(round(linear_vel_value, 2)))
            ax1.legend()

            ax2.plot(linear_vel, linear_vel_very_neg, 'b', linewidth=1.5, label='Very Negative')
            ax2.plot(linear_vel, linear_vel_neg, 'g', linewidth=1.5, label='Negative')
            ax2.plot(linear_vel, linear_vel_zero, 'r', linewidth=1.5, label='Zero')
            ax2.plot(linear_vel, linear_vel_pos, 'c', linewidth=1.5, label='Positive')
            ax2.plot(linear_vel, linear_vel_very_pos, 'm', linewidth=1.5, label='Very Positive')
            ax2.set_title('Acceration Velocity')
            ax2.legend()

        # Rules
        # OR operator means maximum of two AND operator means minimum of two
        acc_x_very_neg_rules = [
            np.fmin(x_error_level_zero, linear_vel_level_very_pos),
            np.fmin(x_error_level_very_neg, linear_vel_level_very_pos),
            np.fmin(x_error_level_very_neg, linear_vel_level_zero),
        ]
        acc_x_neg_rules = [
            np.fmin(x_error_level_pos, linear_vel_level_very_pos),
            np.fmin(x_error_level_pos, linear_vel_level_pos),
            np.fmin(x_error_level_zero, linear_vel_level_pos),
            np.fmin(x_error_level_neg, linear_vel_level_zero),
            np.fmin(x_error_level_very_neg, linear_vel_level_neg),
            np.fmin(x_error_level_very_neg, linear_vel_level_very_neg),
        ]
        acc_x_zero_rules = [
            np.fmin(x_error_level_zero, linear_vel_level_zero),
        ]
        acc_x_pos_rules = [
            np.fmin(x_error_level_very_pos, linear_vel_level_very_pos),
            np.fmin(x_error_level_very_pos, linear_vel_level_pos),
            np.fmin(x_error_level_pos, linear_vel_level_zero),
            np.fmin(x_error_level_zero, linear_vel_level_neg),
            np.fmin(x_error_level_neg, linear_vel_level_neg),
            np.fmin(x_error_level_neg, linear_vel_level_very_neg),
        ]
        acc_x_very_pos_rules = [
            np.fmin(x_error_level_very_pos, linear_vel_level_zero),
            np.fmin(x_error_level_very_pos, linear_vel_level_very_neg),
            np.fmin(x_error_level_zero, linear_vel_level_very_neg),
        ]
        if DEBUG:
            print("####rules#### ")
            print("very_neg_rules", acc_x_very_neg_rules)
            print("neg_rules", acc_x_neg_rules)
            print("zero_rules", acc_x_zero_rules)
            print("pos_rules", acc_x_pos_rules)
            print("very_pos_rules", acc_x_very_pos_rules)

        avg_acc_x_very_neg_rules = np.mean(list(filter(lambda x: x.size > 0, acc_x_very_neg_rules)))
        avg_acc_x_neg_rules = np.mean(list(filter(lambda x: x.size > 0, acc_x_neg_rules)))
        avg_acc_x_zero_rules = np.mean(list(filter(lambda x: x.size > 0, acc_x_zero_rules)))
        avg_acc_x_pos_rules = np.mean(list(filter(lambda x: x.size > 0, acc_x_pos_rules)))
        avg_acc_x_very_pos_rules = np.mean(list(filter(lambda x: x.size > 0, acc_x_very_pos_rules)))
        # avg_acc_x_very_neg_rules = np.min(list(filter(lambda x: x.size > 0, acc_x_very_neg_rules)))
        # avg_acc_x_neg_rules = np.min(list(filter(lambda x: x.size > 0, acc_x_neg_rules)))
        # avg_acc_x_zero_rules = np.min(list(filter(lambda x: x.size > 0, acc_x_zero_rules)))
        # avg_acc_x_pos_rules = np.min(list(filter(lambda x: x.size > 0, acc_x_pos_rules)))
        # avg_acc_x_very_pos_rules = np.min(list(filter(lambda x: x.size > 0, acc_x_very_pos_rules)))

        if DEBUG:
            print("####avg_rules#### ")
            print("very_neg_rules", avg_acc_x_very_neg_rules)
            print("neg_rules", avg_acc_x_neg_rules)
            print("zero_rules", avg_acc_x_zero_rules)
            print("pos_rules", avg_acc_x_pos_rules)
            print("very_pos_rules", avg_acc_x_very_pos_rules)

        # Rule aggregation
        if DEBUG:
            print("#### aggregation ####")
            # print(acc_x_very_neg_rules[0].shape, acc_x_very_neg.shape)
        agg_acc_x_very_neg = np.fmin(avg_acc_x_very_neg_rules, acc_x_very_neg)
        agg_acc_x_neg = np.fmin(avg_acc_x_neg_rules, acc_x_neg)
        agg_acc_x_zero = np.fmin(avg_acc_x_zero_rules, acc_x_zero)
        agg_acc_x_pos = np.fmin(avg_acc_x_pos_rules, acc_x_pos)
        agg_acc_x_very_pos = np.fmin(avg_acc_x_very_pos_rules, acc_x_very_pos)
        acc_x_0 = np.zeros_like(acc_x)

        if SHOW_PLOTS:
            ax3.set_title('Aggregated membership and result (line)')

            ax3.fill_between(acc_x, acc_x_0, agg_acc_x_very_neg,facecolor='b', alpha=0.7)
            ax3.plot(acc_x, acc_x_very_neg, 'b', linewidth=0.5,linestyle='--', label='Very Negative')
            ax3.fill_between(acc_x, acc_x_0, agg_acc_x_neg,facecolor='g', alpha=0.7)
            ax3.plot(acc_x, acc_x_neg, 'g', linewidth=0.5,linestyle='--', label='Negative')
            ax3.fill_between(acc_x, acc_x_0, agg_acc_x_zero,facecolor='r', alpha=0.7)
            ax3.plot(acc_x, acc_x_zero, 'r', linewidth=0.5,linestyle='--', label='Zero')
            ax3.fill_between(acc_x, acc_x_0, agg_acc_x_pos,facecolor='c', alpha=0.7)
            ax3.plot(acc_x, acc_x_pos, 'c', linewidth=0.5,linestyle='--', label='Positive')
            ax3.fill_between(acc_x, acc_x_0, agg_acc_x_very_pos,facecolor='m', alpha=0.7)
            ax3.plot(acc_x, acc_x_very_pos, 'm', linewidth=0.5,linestyle='--', label='Very Positive')

            ax3.legend()

        # Defuzzification
        agg = np.fmax(agg_acc_x_very_neg, np.fmax(agg_acc_x_neg, np.fmax(agg_acc_x_zero, np.fmax(agg_acc_x_pos, agg_acc_x_very_pos))))
        
        try:
            centroid_acc_x = fuzz.defuzz(acc_x, agg, 'centroid')
        except:
            rospy.loginfo('Total area is zero in defuzzification!')
            centroid_acc_x = 0

        deffuz_acc = fuzz.interp_membership(acc_x, agg, centroid_acc_x)  # for plot

        if DEBUG:
            print(centroid_acc_x, slave_linear_x + centroid_acc_x)

        if SHOW_PLOTS:
            # Visualize this
            ax4.plot(acc_x, acc_x_very_neg, 'b', linewidth=0.5, linestyle='--', )
            ax4.plot(acc_x, acc_x_neg, 'g', linewidth=0.5, linestyle='--')
            ax4.plot(acc_x, acc_x_zero, 'r', linewidth=0.5, linestyle='--')
            ax4.plot(acc_x, acc_x_pos, 'c', linewidth=0.5, linestyle='--')
            ax4.plot(acc_x, acc_x_very_pos, 'm', linewidth=0.5, linestyle='--')
            ax4.fill_between(acc_x, acc_x_0, agg, facecolor='Orange', alpha=0.7)
            ax4.plot([centroid_acc_x, centroid_acc_x], [0, deffuz_acc], 'k', linewidth=1.5, alpha=0.9)
            ax4.set_title('Aggregated membership and result (line)')
            ax4.text(centroid_acc_x, 0.5, round(centroid_acc_x, 2),color='k', fontsize=10, ha='center', va='center')

            # Turn off top/right axes
            for ax in (ax0, ax1, ax2, ax3, ax4):
                ax.spines['top'].set_visible(False)
                ax.spines['right'].set_visible(False)
                ax.get_xaxis().tick_bottom()
                ax.get_yaxis().tick_left()
          
        return centroid_acc_x
    
  
    ###### Publish robot velocity ########
    
    vel_msg = Twist()

    ## linear velocity

    vel_inference = linear_vel_inference()
    vel_msg.linear.x = slave_linear_x + vel_inference
    vel_msg.linear.x = vel_msg.linear.x

    ## angular velocity

    _k_a = 1
    _k_l = 1

    global e_angular_z
    if math.fabs(e_linear_x) < 5: # x軸差距小於5時，以leader的r計算
        angular_turn = math.atan2(slave_y, odom_linear_x/odom_angular_z)
    else: # x軸差距大於5時
        angular_turn = math.atan2(slave_y, slave_x)
    if (math.fabs(odom_angular_z) > min_vel_theta): # 有轉動時
        if slave_x > 0 and slave_y > 0: # 第一象限
            if odom_angular_z > 0:  # 逆時針轉
                e_angular_z = e_angular_z + angular_turn 
            else: # 順時針轉
                e_angular_z=e_angular_z- np.pi + angular_turn
        elif slave_x <= 0 and slave_y >= 0: # 第二象限
            if odom_angular_z > 0:
                e_angular_z = e_angular_z - angular_turn
            else:
                e_angular_z = e_angular_z+angular_turn- np.pi 
        elif slave_x < 0 and slave_y < 0: # 第三象限
            if odom_angular_z > 0:
                e_angular_z = e_angular_z - angular_turn
            else:
                e_angular_z = e_angular_z + np.pi + angular_turn
        else: # 第四象限
            if odom_angular_z > 0:
                e_angular_z = e_angular_z - angular_turn
            else:
                e_angular_z = e_angular_z + np.pi + angular_turn
        if(e_angular_z > 4.71 and e_angular_z < 7.85): e_angular_z = e_angular_z-6.28
        if(e_angular_z < -4.71 and e_angular_z > -7.85): e_angular_z = e_angular_z+6.28

    # if ANG_VEL_INFER:
    # if math.fabs(e_angular_z) < 0.2 and math.fabs(e_linear_y) < 0.2:
    #     vel_msg.angular.z = 0
    #     angular_vel_inference()
    # else:
        # ang_infernce = angular_vel_inference()
        # # print("max_vel_theta: ", max_vel_theta)
        # # print("ang_infernce: ", ang_infernce)
        # print("acc_z:  " + str(round(ang_infernce, 2)) + ",    e_y " + str(round(e_linear_y, 2)))
        # if vel_msg.linear.x < -min_vel_x:
        #     if abs(odom_angular_z) > min_vel_theta:
        #             _k_a = -_k_a # _k_a parameter is positive for forward motion
        #             _k_l = -_k_l
        # _k_l = 0
        # _k_a = 1.2
        # vel_msg.angular.z =  1.2 * odom_angular_z + 1 * (slave_angular_z + ang_infernce) + _k_l * e_linear_y + _k_a * math.sin(e_angular_z)
        # vel_msg.angular.z =  (slave_angular_z + ang_infernce)

    # k_l 表示error_y的影響力，k_a表示error_z的影響力
    k_l = 3
    k_a = 1

    _k_a = k_a
    _k_l = k_l
    if (math.fabs(odom_linear_x) < min_vel_x):
        _k_l = 1
        _k_a = 2
    if vel_msg.linear.x < -min_vel_x: # When the slave car is reversing, correct the signs of the parameters to be opposite to those when it is moving forward
        if abs(odom_angular_z) > min_vel_theta:
            _k_a = -k_a # _k_a parameter is positive for forward motion
            _k_l = -k_l

    vel_msg.angular.z =   _k_l * e_linear_y + _k_a * math.sin(e_angular_z)

    # if vel_msg.linear.x > max_vel_x:
    #     vel_msg.linear.x = max_vel_x # Velocity limit
    # elif vel_msg.linear.x < -max_vel_x:
    #     vel_msg.linear.x = -max_vel_x

    # if vel_msg.angular.z  > max_vel_theta:
    #     vel_msg.angular.z = max_vel_theta
    # elif vel_msg.angular.z  < -max_vel_theta:
    #     vel_msg.angular.z = -max_vel_theta

    # print("vel_msg.linear.x: ", vel_msg.linear.x)
    # print("vel_msg.angular.z: ", vel_msg.angular.z)

    if SHOW_PLOTS:
        plt.tight_layout()
        plt.show()

    if PUBLISH_VEL:
        slave_vel_pub.publish(vel_msg)
# Leader robot's velocity and rotaion


def odom_cb(msg):
    global odom_linear_x, odom_linear_y, odom_angular_z
    odom_linear_x = msg.twist.twist.linear.x
    odom_linear_y = msg.twist.twist.linear.y
    odom_angular_z = msg.twist.twist.angular.z
    # print(odom_linear_x, odom_linear_y, odom_angular_z)
    if (math.fabs(odom_linear_x) < min_vel_x):
        odom_linear_x = 0

def slave_cb(msg):
    global slave_linear_x, slave_linear_y, slave_angular_z
    slave_linear_x = msg.linear.x
    slave_linear_y = msg.linear.y
    slave_angular_z = msg.angular.z
    print(slave_linear_x, slave_linear_y, slave_angular_z)


# Call the move function
if __name__ == '__main__':
    # Create ROS node
    rospy.init_node('leader_follower_formation_fuzzy')
    print("Leader Follower Formation Fuzzy Control Node Started")
    leader_robot_name = rospy.get_param('~leader_robot_name', "robot_0")
    follower_robot_name = rospy.get_param('~follower_robot_name', "robot_1")

    base_frame = rospy.get_param("~base_frame", "base_footprint")
    base_to_slave = rospy.get_param("~base_to_slave", "slave1")
    slave_x = rospy.get_param("~slave_x", -1.5)
    slave_y = rospy.get_param("~slave_y", 1.5)
    max_vel_x = rospy.get_param("~max_vel_x", 1.0)
    min_vel_x = rospy.get_param("~min_vel_x", 0.05)
    max_vel_theta = rospy.get_param("~max_vel_theta", 1.0)
    min_vel_theta = rospy.get_param("~min_vel_theta", 0.05)

    # print all parameters
    print("leader_robot_name: ", leader_robot_name)
    print("follower_robot_name: ", follower_robot_name)
    print("base_frame: ", base_frame)
    print("base_to_slave: ", base_to_slave)
    print("slave_x: ", slave_x)
    print("slave_y: ", slave_y)
    print("max_vel_x: ", max_vel_x)
    print("min_vel_x: ", min_vel_x)
    print("max_vel_theta: ", max_vel_theta)
    print("min_vel_theta: ", min_vel_theta)
    rospy.sleep(1)

    # Subscribe to the slave robot's cmd_vel
    slave_vel = rospy.Subscriber( follower_robot_name + "/cmd_vel", Twist, slave_cb)

    # Subscribe to the leader robot's odometry
    leader_vel = rospy.Subscriber(leader_robot_name + "/odom", Odometry, odom_cb)
    


    # Publish to the follower robot's velocity
    print(follower_robot_name + '/cmd_vel')
    slave_vel_pub = rospy.Publisher(follower_robot_name + '/cmd_vel', Twist, queue_size=1)

    # Create a transform listener
    listener = tf.TransformListener()

    # Create a transform broadcaster
    rate = rospy.Rate(10.0)

    # Get tf prefix

    base_frame = tf.TransformListener()
    base_to_slave = tf.TransformListener()

    listner = tf.TransformListener()
    vel_msg = Twist()

    # e_angular_z , e_angular_y
    # e_angular_z = 0
    # e_angular_y = 0

    # Swap coordinates, slave_x and slave_y use leader's coordinate system, 右邊為x軸，前面為y軸
    # 從 leader Relative position 轉 World coordinate
    slave_x = slave_x + slave_y
    slave_y = slave_x - slave_y
    slave_x = -(slave_x - slave_y)

    # All robot stop when shutdown

    def shutdown_cb():
        print("Shutting down")
        try:
            # plot pause
            plt.pause(0.1)

            # close all plot
            plt.close('all')
        except:
            print("No plot to close")
        slave_vel_pub.publish(Twist())

    rospy.on_shutdown(shutdown_cb)
    # Create a Rate object to sleep the loop at a constant rate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Get the transform from the base frame to the slave frame
        try:
            # 追隨者座標與期望座標的相對位置
            # Ex: (追隨者座標) /follower/base_footprint -> (期望座標) /leader/follower
            frame_1 = f"{follower_robot_name}/base_footprint"
            frame_2 = f"{leader_robot_name}/{follower_robot_name}"
            # print(frame_1, frame_2)
            (trans, rot) = listener.lookupTransform(frame_1, frame_2, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Waiting for TF data or no TF data available")
            rospy.sleep(1)
            continue

        # Quaternion to Euler angles
        rpy = tf.transformations.euler_from_quaternion(rot)

        e_angular_x, e_angular_y, e_angular_z = rpy
        e_linear_x, e_linear_y, e_linear_z = trans
        e_angular_z = e_angular_z
        try:
            move()
        except rospy.ROSInterruptException:
            pass

        rate.sleep()
