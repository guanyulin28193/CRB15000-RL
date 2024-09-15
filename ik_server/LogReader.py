import re

def extract_value_function_rewards(log_file_path):
    # Regular expression to match the "Value Function Reward" line
    reward_pattern = re.compile(r"Value Function Reward: ([\d.]+)")
    rewards = []
    lines = 0
    
    # Read the log file and extract rewards
    with open(log_file_path, 'r') as file:
        for line in file:
            lines += 1
            match = reward_pattern.search(line)
            if match:
                
                reward = float(match.group(1))
                rewards.append(reward)
    print(f"Extracted {len(rewards)} rewards from {lines} lines.")
    # If no rewards are found, return None
    if not rewards:
        return None, None
    
    # Calculate min and max rewards
    min_reward = min(rewards)
    max_reward = max(rewards)
    
    return min_reward, max_reward

# Usage example:
log_file_path = 'C:/Users/18125/CRB15000-RL/results/GraspVF/run_logs/Player-0.log'
min_reward, max_reward = extract_value_function_rewards(log_file_path)
print(f"Minimum Value Function Reward: {min_reward}")
print(f"Maximum Value Function Reward: {max_reward}")
