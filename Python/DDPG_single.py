import numpy as np
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from DDPG_agent import Agent
from IK import calculate_angles
import torch
from Unity_env import env_reset, env_next_step
#from torch.utils.tensorboard import SummaryWriter


if __name__ == '__main__':
    engine = EngineConfigurationChannel()
    engine.set_configuration_parameters(width=640, height=480, time_scale=3, quality_level=0)
    env = UnityEnvironment(file_name="C:/Users/18125/CRB15000-RL/TrainBuild/ABB-RL.exe",
                           side_channels=[engine], no_graphics=True)
    env.reset()

    #####################################################
    # Set the number of episodes to run the training
    num_episodes=10000
    max_steps= 1000
    #####################################################
    
    # Get env information
    behavior_name = list(env.behavior_specs.keys())[0]
    behavior_value = list(env.behavior_specs.values())[0]
    print("Name of the behavior : ", behavior_name)
    
    discrete_actions = None
    total_steps = 0
    stepsNum = 0
    scores_average_window=100
    episode_scores = []
    obs_space_size = behavior_value.observation_specs[0].shape[0]
    act_space_size = behavior_value.action_spec.continuous_size
    print("obs_space_size: ", obs_space_size)
    print("act_space_size: ", act_space_size)
    
    
    DDPG_agent= Agent(state_size=obs_space_size, action_size=act_space_size, num_agents=1, random_seed=0)
    

    #training loop
    for i_episode in range(1, num_episodes+1):
        #reset the environment
        ep_reward = 0
        state = env_reset(env, behavior_name)

        while True:
            action = DDPG_agent.act(state)
            action= calculate_angles([action[0][0], action[0][1], action[0][2]],[action[0][3], action[0][4], action[0][5]])
            #print(action)
            nest_state, reward, done = env_next_step(env, behavior_name, action)
            DDPG_agent.step(state, action, reward, nest_state, done)

            state = nest_state
            reward = np.squeeze(reward, axis=0)
            ep_reward += reward
            if done:
                break
        print("Episode: ", i_episode, "Reward: ", ep_reward)
        episode_scores.append(ep_reward)
        average_score = np.mean(episode_scores[i_episode-min(i_episode,scores_average_window):i_episode+1])
        #Print current and average score
        #print('\nEpisode {}\tMax Score: {:.2f}\tAverage Score: {:.2f}'.format(i_episode, episode_scores[i_episode-1], average_score), end="")
        
        # Save trained Actor and Critic network weights for agent
        an_filename = "ddpgActor_Model.pth"
        torch.save(DDPG_agent.actor_local.state_dict(), an_filename)
        cn_filename = "ddpgCritic_Model.pth"
        torch.save(DDPG_agent.critic_local.state_dict(), cn_filename)

            # Check to see if the task is solved (i.e,. avearge_score > solved_score over 100 episodes). 
    # If yes, save the network weights and scores and end training.
        if i_episode > 100 and average_score >= 0.5:
            print('\nEnvironment solved in {:d} episodes!\tAverage Score: {:.2f}'.format(i_episode, average_score))

            # Save the recorded Scores data
            scores_filename = "ddpgAgent_Scores.csv"
            np.savetxt(scores_filename, episode_scores, delimiter=",")
            break
    env.close()