from dm_control import mjcf
from robot import Rabbit
from dm_control import mujoco
class Floor():
    """
    A simple flat terrain
    """
    def __init__(self, path = "floor.xml"):
        self.mjcf_model = mjcf.from_path(path)
        pass
    
    def randomize(self):
        raise NotImplementedError
    
    def spawn(self, robot, init_pos=[0,0,0], init_ori=[1,1,1,1]):
        """
        attaches robot mjcf model to world mjcf model
        arguments:
        robot - object of class Robot() from robot.py
        init_pos - tuple of size 3
        init_ori - tuple of size 4
        TODO_ Orientation Options
        """
        attch = self.mjcf_model.find('site', 'attachment_site')
        attch.pos = init_pos
        attch.quat = init_ori
        print(attch)
        attch.attach(robot.mjcf_model)
        # attch.find('body','rabbit/torso').quat=[0,0,1,0]
        # attch.find('body','rabbit/torso').pos=[0,0,10]
        pass
    
if(__name__ == "__main__"):
    f = Floor()
    robo = Rabbit()
    f.spawn(robo, [0,0,1.115])
    # print(f.mjcf_model.find('body','rabbit/torso').pos, f.mjcf_model.find('body','rabbit/torso').quat)    
    physics = mujoco.Physics.from_xml_string(f.mjcf_model.to_xml_string())
    # print(physics)
    # print(f.mjcf_model.to_xml_string())