import numpy as np
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise





if __name__ == '__main__':
    engine = EngineConfigurationChannel()
    engine.set_configuration_parameters(width=640, height=480, time_scale=5, quality_level=0)
    env = UnityEnvironment(file_name="C:/Users/18125/CRB15000-RL/TrainBuildLow/ABB-RL.exe",
                           side_channels=[engine], no_graphics=True)
    env = UnityToGymWrapper(env)

    n_actions = env.action_space.shape[-1]
    print("Number of actions: ", n_actions)
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=1, tensorboard_log="./ddpg_low_tensorboard/")
    model.learn(total_timesteps=10000, log_interval=10)
    model.save("ddpg_low")
    vec_env = model.get_env()

    del model # remove to demonstrate saving and loading

    model = DDPG.load("ddpg_low")
    model.learn(total_timesteps=10000, log_interval=10)

    obs = vec_env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = vec_env.step(action)
        if dones.any():
            obs = vec_env.reset()