def reward_function(params):

    if params['all_wheels_on_track']:
        reward = params['progress']
    else:
        return float(0.001)

    # Speed is really good
    reward *= math.exp(2 * speed)

    return float(reward)