import gymnasium as gym
import numpy as np
from gymnasium import spaces
from mlagents_envs.environment import ActionTuple

class UnityToGymWrapper(gym.Env):
    def __init__(self, unity_env):
        self.unity_env = unity_env
        self.behavior_name = list(self.unity_env.behavior_specs.keys())[0]
        self.behavior_value = list(self.unity_env.behavior_specs.values())[0]
        self.obs_space_size = self.behavior_value.observation_specs[0].shape[0]
        self.act_space_size = self.behavior_value.action_spec.continuous_size
        self.action_space = spaces.Box(low=-1, high=1, shape=(self.act_space_size,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.obs_space_size,), dtype=np.float32)

    def reset(self,seed=None):
        self.unity_env.reset()
        dec, term = self.unity_env.get_steps(self.behavior_name)
        done = len(term.agent_id) > 0
        state = term.obs[0] if done else dec.obs[0]
        return state, {}
    
    def step(self, action):
        action_tuple = ActionTuple()
        action_tuple.add_continuous(np.array([action]))
        self.unity_env.set_actions(self.behavior_name, action_tuple)
        self.unity_env.step()

        dec, term = self.unity_env.get_steps(self.behavior_name)
        done = len(term.agent_id) > 0
        #print("done: ", done) if done else None
        reward = term.reward if done else dec.reward
        next_state = term.obs[0] if done else dec.obs[0]

        # Assuming that 'terminated' is equivalent to 'done' in your case
        terminated = done

        # Assuming that 'truncated' is False as it's not clear from your code what it should be
        truncated = False

        # Assuming that 'info' is an empty dictionary as it's not clear from your code what it should be
        info = {}

        return next_state, reward, terminated, truncated, info
    
    def close(self):
        self.unity_env.close()