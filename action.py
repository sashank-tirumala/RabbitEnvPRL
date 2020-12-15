import numpy as np
from pmtg_controller import WalkingController
import env
def apply_pd_control(motor_commands, motor_vel_commands, qpos_act, qvel_act, kp=200,kd=10):
        '''
        Apply PD control to reach desired motor position commands
        Ret:
            applied_motor_torque : array of applied motor torque values in order [FLH FLK FRH FRK BLH BLK BRH BRK FLA FRA BLA BRA]
        '''
        applied_motor_torque = kp * (motor_commands - qpos_act) + kd * (motor_vel_commands - qvel_act)
        applied_motor_torque = applied_motor_torque.tolist()
        return applied_motor_torque

def pmtg_action(action, walkcon, physics):
    """
    Should directly return motor torques for the simulator to use
    """
    kp = 300
    kd = 10
    dt = physics.timestep()
    qpos = physics.joint_angles()
    qvel = physics.joint_vel()
    leg_m_angle_cmd, X, Y = walkcon.run_pmtg_traj_rabbit(action, dt)
    m_angle_cmd_ext = np.array(leg_m_angle_cmd)
    m_vel_cmd_ext = np.zeros(4)
    applied_motor_torque = apply_pd_control(m_angle_cmd_ext, m_vel_cmd_ext, qpos, qvel, kp=kp,kd=kd)
    applied_motor_torque = np.clip(applied_motor_torque, -60, 60)
    return applied_motor_torque

if(__name__ == "__main__"):
    physics = env.Physics.from_xml_string(env.get_model())
    action = [2.5,2.5,-0.1,-0.1,0.5,0.5,0,0,0,0]
    walkcon = WalkingController([0, np.pi])
    print(pmtg_action(action, walkcon, physics))
    pass