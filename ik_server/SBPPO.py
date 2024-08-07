import numpy as np
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from Unity_Gym import UnityToGymWrapper
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.callbacks import BaseCallback


############################################################################################################
# Set TestMode to True to test the model, False to train the model
TestMode=False

############################################################################################################

def test_model(model, env, num_episodes=100):
    rewards = []
    for i in range(num_episodes):
        obs = env.reset()
        done = False
        episode_reward = 0
        while not done:
            action, _ = model.predict(obs)
            #print(action)
            obs, reward, done, info = env.step(action)
            episode_reward += reward
        rewards.append(episode_reward)
        print(f"Episode {i + 1} reward: {episode_reward}")
    print(f"Average reward over {num_episodes} episodes: {sum(rewards) / num_episodes}")

if __name__ == '__main__':
    try:
        engine = EngineConfigurationChannel()
        engine.set_configuration_parameters(width=640, height=480, time_scale=(1 if TestMode else 300), quality_level=0)
        Unity_env = UnityEnvironment(file_name="C:/Users/18125/CRB15000-RL/TrainBuildLow/ABB-RL.exe",
                            side_channels=[engine], no_graphics=(not TestMode))
        Unity_env.reset()
        env = UnityToGymWrapper(Unity_env)
        # Create environment
        env = DummyVecEnv([lambda: env])  # replace 'env' with your environment

        # Define policy network configuration
        policy_kwargs = dict(
            net_arch=[dict(pi=[256, 256, 256], vf=[256, 256, 256])],  # This creates an actor-critic network with 3 layers of 256 units each
        )

        # Create PPO model
        model = PPO(
            "MlpPolicy",
            env,
            verbose=2,
            tensorboard_log="./ppo_tensorboard/",
            policy_kwargs=policy_kwargs,
            learning_rate=0.0003,  # Def: 0.0003 (1e-5 to 1e-3)
            n_steps=40960 // 10,  # Number of steps to run for each environment per update (buffer_size / num_epoch)
            batch_size=4096,  # Minibatch size
            n_epochs=8,  # Number of epochs to use for each update
            gamma=0.995,  # Discount factor
            gae_lambda=0.90,  # Factor for trade-off of bias vs variance for Generalized Advantage Estimator
            clip_range=0.2,  # Clipping parameter for PPO
        )
        
        if TestMode:
            model.load("C:/Users/18125/CRB15000-RL/PPOmodels/ppo_model_2500000_steps.zip")
            test_model(model, env, num_episodes=100)
        else:
            # Define checkpoint callback
            checkpoint_callback = CheckpointCallback(save_freq=5e4, save_path='./PPOmodels/',name_prefix='ppo_model')
            #model.load("./PPOmodels/ppo_model_100000_steps.zip")
            # Train model
            model.learn(total_timesteps=int(5e6), callback=[checkpoint_callback])
            model.save("./PPOmodels/ppo_model_final.zip")

    except KeyboardInterrupt:
        print("Interrupted by user, closing environment.")
        model.save("./PPOmodels/ppo_model_interrupt.zip")
        env.close()