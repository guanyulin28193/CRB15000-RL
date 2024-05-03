import numpy as np
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from stable_baselines3 import DDPG
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.env_checker import check_env
from gymnasium import spaces



if __name__ == '__main__':
    engine = EngineConfigurationChannel()
    engine.set_configuration_parameters(width=640, height=480, time_scale=0.5, quality_level=0)
    env = UnityEnvironment(file_name="C:/Users/18125/CRB15000-RL/TrainBuild/ABB-RL.exe",
                           side_channels=[engine], no_graphics=True)
    #Gym_env = UnityToGymWrapper(env)
    #print(Gym_env.action_space)
