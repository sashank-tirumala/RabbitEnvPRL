# author = @stsashank6@gmail.com
# ============================================================================

"""Rabbit Domain."""
import collections

from dm_control import mujoco
from dm_control.rl import control
from dm_control.suite import base
from dm_control.suite.utils import randomizers
from dm_control.utils import containers
from dm_control.utils import rewards
from dm_control import viewer
from pyquaternion import Quaternion

import world
import robot
import state

import numpy as np
import PIL.Image


_DEFAULT_TIME_LIMIT = 25
_CONTROL_TIMESTEP = .02 #Same as n_frame_skip in pybullet
INIT_POS = [0,0,0.05]
INIT_ORI= [1, 0, 0, 0]
SUITE = containers.TaggedTasks()


def get_model():
    robo = robot.Rabbit()
    arena = world.Floor()
    arena.spawn(robo, INIT_POS, INIT_ORI)
    return arena.mjcf_model.to_xml_string()

def run(time_limit=_DEFAULT_TIME_LIMIT, random=None, environment_kwargs=None):
  """Returns the Run task."""
  physics = Physics.from_xml_string(get_model())
  task = Humanoid(random=random)
  environment_kwargs = environment_kwargs or {}
  return control.Environment(
      physics, task, time_limit=time_limit, control_timestep=_CONTROL_TIMESTEP,
      **environment_kwargs)

class Physics(mujoco.Physics):
  """Physics simulation with additional features for the Rabbit domain."""

  def center_of_mass_position(self):
    """Returns position of the center-of-mass."""
    return self.named.data.subtree_com['rabbit/torso'].copy()

  def center_of_mass_velocity(self):
    """Returns the velocity of the center-of-mass."""
    return self.named.data.sensordata['rabbit/torso_subtreelinvel'].copy()

  def joint_angles(self):
    """Returns the state without global orientation or position
    The order of the data is Right thigh, Right knee, Left thigh, Left knee"""
    return self.data.qpos[3:].copy()  # Skip the 3 DoFs of the free root joint.

  def torso_angle(self):
    """Returns Torso Angle with vertical axis in degrees. Upright is 0 degrees"""
    quat = Quaternion(matrix = np.reshape(np.array(self.named.data.geom_xmat['rabbit/torso_geom']).copy(),[3,3]))
    angle = quat.degrees
    if(np.linalg.norm(np.array(quat.axis) - np.array([0,-1,0]))<1e-3):
        angle = -1*angle
    return angle
  
  def joint_vel(self):
      """ Returns the velocity of the joints. The order of data is again Right Thigh, Right Knee, Left Thigh, Left Knee"""
      return self.data.qvel[3:].copy()
    # return Quaternion(np.array(self.named.data.find('body', 'rabbit/torso').quat.copy())).degrees
  
  def contact_points_and_force(self):
    """
    The order is right-leg position, right-leg force, left-leg position, left-leg force
    """
    pos_r = self.named.data.site_xpos['rabbit/t1'].copy() - self.named.data.subtree_com['rabbit/torso'].copy()
    pos_l = self.named.data.site_xpos['rabbit/t2'].copy() - self.named.data.subtree_com['rabbit/torso'].copy()
    return np.concatenate([pos_r,physics.named.data.sensordata['rabbit/s_t1'], pos_l, physics.named.data.sensordata['rabbit/s_t2']])


class Humanoid(base.Task):
  """A humanoid task."""

  def __init__(self, random=None):
    """Initializes an instance of `Humanoid`.
    Args:
      move_speed: A float. If this value is zero, reward is given simply for
        standing up. Otherwise this specifies a target horizontal velocity for
        the walking task.
      pure_state: A bool. Whether the observations consist of the pure MuJoCo
        state or includes some useful features thereof.
      random: Optional, either a `numpy.random.RandomState` instance, an
        integer seed for creating a new `RandomState`, or None to select a seed
        automatically (default).
    """
    super().__init__(random=random)

  def initialize_episode(self, physics):
    """Sets the state of the environment at the start of each episode.
    Args:
      physics: An instance of `Physics`.
    """
    # Find a collision-free random initial configuration.
    super().initialize_episode(physics)

  def get_observation(self, physics):
    """Returns either the pure state or a set of egocentric features."""
    obs = state.obs1(physics)
    # print(physics.contact_points_and_force())
    return obs

  def get_reward(self, physics):
    """Returns a reward to the agent."""
    return 1
  
  def get_termination(self, physics):
    print(physics.torso_angle())
    if(physics.torso_angle() > 90 or physics.torso_angle() < -90):
      print("in termination")
      return -10
    return None
  


if(__name__ == "__main__"):
    physics = Physics.from_xml_string(get_model())
    def random_policy(time_step):
        vals = np.random.uniform(low=-20,
                           high=20,
                           size=4)
        return vals
    # physics = Physics.from_xml_path("rabbit_new.xml")
    # pixels = physics.render()
    # im = PIL.Image.fromarray(pixels)
    # im.show()
    env = run()
    viewer.launch(env, policy=random_policy)
