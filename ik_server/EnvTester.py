import numpy as np
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from Unity_Gym import UnityToGymWrapper

if __name__ == '__main__':
    engine = EngineConfigurationChannel()
    engine.set_configuration_parameters(width=640, height=480, time_scale=1, quality_level=0)
    env = UnityEnvironment(file_name="C:/Users/18125/CRB15000-RL/TrainBuild/ABB-RL.exe",
                           side_channels=[engine])
    env.reset()
    env = UnityToGymWrapper(env)
    #print(env.reset())
    for i in range(21):
        print(i)
        env.step([-1, 1, -0.5, 1, 0, 0.5])
    env.close()