import pyrobolearn as prl 
sim = prl.simulators.Mujoco()
world = prl.worlds.BasicWorld(sim)
rabbit = worlds.load_robot
while True:
    sim.render()