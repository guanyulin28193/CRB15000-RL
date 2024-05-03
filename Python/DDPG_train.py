import numpy as np
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
from DDPG_agent import Agent
from IK import calculate_angles
import torch
#from stable_baselines3 import PPO
#from mlagents_envs.envs.unity_gym_env import UnityToGymWrapper


if __name__ == '__main__':
    engine = EngineConfigurationChannel()
    engine.set_configuration_parameters(width=640, height=480, time_scale=0.5, quality_level=0)
    env = UnityEnvironment(file_name="C:/Users/18125/CRB15000-RL/TrainBuild/ABB-RL.exe",
                           side_channels=[engine])
    env.reset()

    # Set the number of episodes to run the training
    num_episodes=50
    episode_scores = []
    scores_average_window = 100      
    solved_score = 1.0 
    max_steps= 5000
    
    # Get env information
    behavior_names = list(env.behavior_specs.keys())[0]
    behavior_value = list(env.behavior_specs.values())[0]
    print("Name of the behavior : ", behavior_names)
    
    discrete_actions = None
    total_steps = 0
    stepsNum = 0
    obs_space_size = behavior_value.observation_specs[0].shape[0]
    act_space_size = behavior_value.action_spec.continuous_size
    print("obs_space_size: ", obs_space_size)
    print("act_space_size: ", act_space_size)
    
    steps = env.get_steps(behavior_names)
    num_agents = len(steps[0].agent_id)
    print('Number of Agents: ', num_agents)

    #create agents
    agent_list = list(range(num_agents))
    for agent in range(len(steps[0].agent_id)):
        agent_list[agent] = Agent(state_size=obs_space_size, action_size=act_space_size, num_agents=1, random_seed=0)
    #print(agent_list)

    #training loop
    for i_episode in range(1, num_episodes+1):
        #reset the environment
        env.reset()
        agent_scores = np.zeros(num_agents)
        for agent in agent_list:
            agent.reset()

        steps = env.get_steps(behavior_names)
        while True:
            max_steps -= 1
            actions = []
            for agent in range(len(steps[0].agent_id)):
                action = agent_list[agent].act(steps[0].obs[0][agent])
                action = action[0]
                #get IK from solver with positions and orientations
                actions.append(calculate_angles([action[0], action[1], action[2]],[action[3], action[4], action[5]]))

            agent_scores += steps[0].reward
            
            action_tuple = ActionTuple()
            action_tuple.add_continuous(np.array(actions))
            env.set_actions("CRB15000?team=0", action_tuple)
            next_steps = env.get_steps(behavior_names)
            
            for agent in range(len(steps[0].agent_id)):
                agent_list[agent].step(steps[0].obs[0][agent], actions[agent], steps[0].reward[agent], next_steps[0].obs[0][agent], False)
            steps = next_steps
            if max_steps <= 0:
                break

        # Add episode score to Scores and...
        # Calculate mean score over last 100 episodes 
        # Mean score is calculated over current episodes until i_episode > 100
        episode_scores.append(np.max(agent_scores))
        average_score = np.mean(episode_scores[i_episode-min(i_episode,scores_average_window):i_episode+1])

        #Print current and average score
        print('\nEpisode {}\tMax Score: {:.2f}\tAverage Score: {:.2f}'.format(i_episode, episode_scores[i_episode-1], average_score), end="")
    

        # Check to see if the task is solved (i.e,. avearge_score > solved_score over 100 episodes). 
        # If yes, save the network weights and scores and end training.
        if i_episode > 100 and average_score >= solved_score:
            print('\nEnvironment solved in {:d} episodes!\tAverage Score: {:.2f}'.format(i_episode, average_score))

            # Save the recorded Scores data
            scores_filename = "ddpgAgent_Scores.csv"
            np.savetxt(scores_filename, episode_scores, delimiter=",")
            break



        
        
        



    """for i in range(20):
        steps = env.get_steps("CRB15000?team=0")
        actions = []
        for agent in range(len(steps[0].agent_id)):
            print(agent)
            #print(steps[0].reward[agent])
            #print(steps[0].obs[0][agent])
            # 0-24 , 25 actions
            actions.append([40, 30, 60, 1, 0, 0.5]) 
        #print(actions)
        action_tuple = ActionTuple()
        action_tuple.add_continuous(np.array(actions))
        env.set_actions("CRB15000?team=0", action_tuple)
        env.step()"""



    
