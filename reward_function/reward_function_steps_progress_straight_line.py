# fp-steph-v15
def reward_function(params):

    center_lane = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169] #Fill in the waypoints

    # Read input parameters
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering = abs(params['steering_angle']) # Only need the absolute steering angle

    reward = 0
    if params["closest_waypoints"][1] in center_lane:
        # Calculate 3 marks that are farther and father away from the center line
        marker_1 = 0.1 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.5 * track_width

        # Give higher reward if the car is closer to center line and vice versa
        if distance_from_center <= marker_1:
            reward = 10
        elif distance_from_center <= marker_2:
            reward = 5
        elif distance_from_center <= marker_3:
            reward = 1
        else:
            reward = 1e-3  # likely crashed/ close to off track

        # Steering penality threshold, change the number based on your action space setting
        ABS_STEERING_THRESHOLD = 10 

        # Penalize reward if the car is steering too much
        if steering > ABS_STEERING_THRESHOLD:
            reward *= 0.8
    else:

        if params["all_wheels_on_track"] and params["steps"] > 0:
            reward = ((params["progress"] / params["steps"]) * 100) + (params["speed"]**2)
        else:
            reward = 1e-3
        
    return float(reward)