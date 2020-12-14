from dm_control import mjcf

class Rabbit():
    def __init__(self, path = "rabbit_new_local.xml"):
        self.mjcf_model = mjcf.from_path(path)
        self.name = 'rabbit'
        self.com_name = 'torso'

    def randomize(self):
        raise NotImplementedError
    

if(__name__ == "__main__"):
    robo = Rabbit()
    robo.mjcf_model.worldbody.body['torso'].pos = [0, 0, 5]
    print(robo.mjcf_model.worldbody.body['torso'].pos)