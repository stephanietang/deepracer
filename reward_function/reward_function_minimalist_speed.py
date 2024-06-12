import math

def reward_function(params):

    speed = params['speed']

    if params['all_wheels_on_track']:
        # Speed is really good
        reward = params['progress'] + math.exp(2 * speed)
    else:
        return float(0.001)

    return float(reward)