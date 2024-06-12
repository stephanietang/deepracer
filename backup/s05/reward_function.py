def reward_function(params):
    
    all_wheels_on_track = params['all_wheels_on_track']
    speed = params['speed']
    track_witdh = params['track_width']
    distance_from_center = params['distance_from_center']
    progress = params['progress']
    steps = params['steps']

    # Huge penalties
    if not all_wheels_on_track:
        return float(1e-3)

    # Speed Reward
    speed_reward = 0
    if speed > 3.6:
        speed_reward = speed * 2
    else:
        speed_reward = 1e-3
    
    # Center the car
    track_width_half = track_witdh / 2
    quadratic_center = (distance_from_center / track_width_half) ** 4
    distance_reward = 1 - quadratic_center

    # Give a higher reward if the car is making progress towards the finish line
    step_award = (progress / steps) * 100
        
    # Give a higher reward if the car completes the track in the allotted time
    finish_award = 0
    if progress == 100:
        finish_award = 2
    
    reward = 0
    reward += speed_reward
    reward += distance_reward
    reward += step_award
    reward += finish_award

    print('progress: {}, steps: {}, speed: {}, distance_from_center {}'.format(progress, steps, speed, distance_from_center))
    print('speed_reward: {}, distance_reward: {}, step_award: {}, finish_award: {}, reward: {}'.format(speed_reward, distance_reward, step_award, finish_award, reward))
    
    return float(reward)