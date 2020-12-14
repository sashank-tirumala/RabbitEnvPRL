import numpy as np
def obs1(physics):
    """
    Basic large observation
    """
    joint_angles = np.array(physics.joint_angles())
    joint_velocities = np.array(physics.joint_vel())
    torso_angle = np.array([physics.torso_angle()])
    com_velocity = np.array(physics.center_of_mass_velocity())
    obs = np.concatenate([joint_angles, joint_velocities, torso_angle, com_velocity]).ravel()
    return obs