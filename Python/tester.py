import numpy as np
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

if __name__ == '__main__':
    engine = EngineConfigurationChannel()
    engine.set_configuration_parameters(width=640, height=480, time_scale=1, quality_level=0)
    env = UnityEnvironment(file_name="C:/Users/18125/CRB15000-RL/TrainBuildLow/ABB-RL.exe",
                           side_channels=[engine])
    env.reset()

    keys = env.behavior_specs.keys()
    for i in range(21):
        print(i)
        steps = env.get_steps("CRB15000?team=0")
        #print(steps)
        #print(steps[0].obs)
        print(len(steps[1].obs[0])==1)
        print(steps[0].agent_id)

        # 0-24 , 25 actions
        action_tuple = ActionTuple()
        action_tuple.add_continuous(np.array([[1, 1, 1, 1, 1, 1]]))  # Normalized -1 to 1

        env.set_actions("CRB15000?team=0", action_tuple)
        env.step()

    env.reset()