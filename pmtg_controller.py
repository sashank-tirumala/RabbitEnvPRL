"""Utilities for realizing walking controllers."""
import sys, os
sys.path.append(os.path.realpath('../..'))

from utils.ik_class import Serial2RKin
import numpy as np
import math
import matplotlib.pyplot as plt

PI = np.pi
no_of_points = 100


def constrain_theta(theta):
    theta = np.fmod(theta, 2 * no_of_points)
    if (theta < 0):
        theta = theta + 2 * no_of_points
    return theta

class WalkingController():
    def __init__(self,phase=[0, 0]):
        self.theta={'left':0,'right':no_of_points}
        self.MOTOROFFSETS_Rabbit = [np.pi/2, 0]
        self.THIGH_JOINT_LIMITS_Rabbit = [math.radians(90),math.radians(250)]
        self.KNEE_JOINT_LIMITS_Rabbit = [math.radians(0),math.radians(150)]
        self.Rabbit_Kin = Serial2RKin(base_pivot=[0,0],link_lengths=[0.2,0.4249])

    def apply_fk(self, actual_angles):
        actual_angles = actual_angles - self.MOTOROFFSETS_Rabbit
        [x,y] = self.Rabbit_Kin.forwardKinematics(actual_angles)
        return [x,y]

    def run_pmtg_traj_rabbit(self, action=0, dt=0.01):
        '''
        Semi-elliptical trajectory controller
        Args:
            theta  : trajectory cycle parameter theta
            action : trajectory modulation parameters predicted by the policy
            [freql,freqr,heightl,heightr,sl_l,sl_r,offsetx_l,offset_r,offsety_l,offsety_r]
        Ret:import sys, os
sys.path.append(os.path.realpath('../..'))
            leg_motor_angles : list of motors positions for the desired action [FLH FLK FRH FRK BLH BLK BRH BRK FLA FRA BLA BRA]
        '''
        # action = [2.5,2.5,-0.1,-0.1,0.5,0.5,0,0,0,0]
        y_center = 0.5
        foot_clearance = -0.1
        valid_list = []
        X = []
        Y = []
        left_leg = {'name':'left', 'freq':action[0], 'height':action[2], 'step_length':action[4], 'x_offset':action[6], 'y_offset':action[8], 'theta': self.theta['left']}
        right_leg = {'name':'right', 'freq':action[1], 'height':action[3], 'step_length':action[5], 'x_offset':action[7], 'y_offset':action[9], 'theta': self.theta['right']}
        legs = [right_leg, left_leg]
        leg_motor_angles=[]
        for leg in legs:
            leg_theta = (leg['theta']/ (2 * no_of_points)) * 2 * PI
            x = -leg['step_length'] * np.cos(leg_theta)/2 + leg['x_offset'] #+0.2
            if leg_theta > PI:
                flag = 0
            else:
                flag = 1
            y = leg['height'] * np.sin(leg_theta) * flag + y_center + leg['y_offset']
            X.append(x)
            Y.append(y)
            valid, motor_hip, motor_knee = self.Rabbit_Kin.inverseKinematics([x, y])
            # print(valid, motor_hip, motor_knee)
            motor_hip = motor_hip + self.MOTOROFFSETS_Rabbit[0]
            motor_knee = motor_knee + self.MOTOROFFSETS_Rabbit[1]
            leg_motor_angles.append(motor_hip)
            leg_motor_angles.append(motor_knee)
            self.theta[leg['name']]= constrain_theta(self.theta[leg['name']] + 2*no_of_points*leg['freq']*dt)
        return leg_motor_angles, X, Y

if (__name__ == "__main__"):
    walkcon = WalkingController()
    x_tot = []
    y_tot = []
    for i in range(2000):
        action = [2.5,2.5,-0.1,-0.1,0.15,0.15,0,0,0,0]
        ang, x, y = walkcon.run_pmtg_traj_rabbit(action, 0.01)
        x_tot.append(x[0])
        y_tot.append(y[0])

    plt.figure()
    plt.plot(x_tot, y_tot,'r.')
    plt.show()