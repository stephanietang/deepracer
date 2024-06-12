import math

def reward_function(params):
    # Reward for completing the track
    if params['progress'] == 100:
        reward = 100

        print('finish the track')

    # Penalize for driving off track
    elif not params['all_wheels_on_track']:
        reward = (1e-3)

    else:
        # Get params
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        track_width = params['track_width']
        distance_from_center = params['distance_from_center']
        
        # {0 < x < 1} Reward for target number of steps
        STEP_GOAL_PER_LAP = 55
        step_goal_right_now = progress * STEP_GOAL_PER_LAP
        if progress != 0:
            step_reward = steps / step_goal_right_now
        else:
            step_reward = (1e-3)
        
        # {0 < x < 1} Reward for high speed
        if speed > 3.6:
            speed_reward = speed * 2
        else:
            speed_reward = (1e-3)
                            
        # {0 < x < 1} Penalize for getting too far from the center
        track_width_half = track_width / 2
        quadratic_center = (distance_from_center / track_width_half) ** 4
        distance_reward = 1 - quadratic_center

        # Standard process steps reward
        standard_reward = progress / steps

        # Combine Rewards
        reward = 0
        reward += step_reward
        reward += speed_reward
        reward += distance_reward
        reward += standard_reward

        # Speed is really good
        reward *= math.exp(2 * speed)

    return float(reward)