import pandas as pd


def reward_function(params):

    # Actual reward function go here
    # from reward_function_progress_velocity import reward_function
    from reward_function_optimal_trace5 import reward_function
    reward = reward_function(params)
    return float(reward)


# Sample params, not real, only for testing in python, do not copy to AWS
offtrack = {'all_wheels_on_track': True,
            'track_width': 3,
            'distance_from_center': 1.6,
            'is_left_of_center': True,
            'heading': 0,
            'progress': 7.2,
            'steering_angle': -20,
            'speed': 5.33,
            'x': 3.12,
            'y': 1.15,
            'steps': 12,
            'waypoints': [1.21, 0.26],
            'closest_waypoints': 2,
            'is_offtrack': True,
            'is_crashed': False
            }

params2 = {'all_wheels_on_track': True,
           'track_width': 3,
           'distance_from_center': 0.5,
           'is_left_of_center': False,
           'heading': 0,
           'progress': 8.2,
           'steering_angle': 10,
           'speed': 2.67,
           'x': 3.12,
           'y': 1.15,
           'steps': 5,
           'waypoints': [1.21, 0.26],
           'closest_waypoints': 2,
           'is_offtrack': False,
           'is_crashed': False
           }

params3 = {'all_wheels_on_track': True,
           'track_width': 3,
           'distance_from_center': 0.3,
           'is_left_of_center': True,
           'heading': 0,
           'progress': 10,
           'steering_angle': -10,
           'speed': 8,
           'x': 3.12,
           'y': 1.15,
           'steps': 12,
           'waypoints': [1.21, 0.26],
           'closest_waypoints': 2,
           'is_offtrack': False,
           'is_crashed': False
           }

params4 = {'all_wheels_on_track': True,
           'track_width': 3,
           'distance_from_center': 0.1,
           'is_left_of_center': True,
           'heading': 0,
           'progress': 10.2,
           'steering_angle': 0,
           'speed': 5.33,
           'x': 3.12,
           'y': 1.15,
           'steps': 12,
           'waypoints': [1.21, 0.26],
           'closest_waypoints': 2,
           'is_offtrack': False,
           'is_crashed': False
           }

params5 = {'all_wheels_on_track': True,
           'track_width': 3,
           'distance_from_center': 1.2,
           'is_left_of_center': True,
           'heading': 0,
           'progress': 10,
           'steering_angle': -30,
           'speed': 5.33,
           'x': 3.12,
           'y': 1.15,
           'steps': 12,
           'waypoints': [1.21, 0.26],
           'closest_waypoints': 2,
           'is_offtrack': False,
           'is_crashed': False
           }

params6 = {'all_wheels_on_track': True,
           'track_width': 3,
           'distance_from_center': 1.2,
           'is_left_of_center': False,
           'heading': 0,
           'progress': 10.9,
           'steering_angle': 30,
           'speed': 5.33,
           'x': 3.12,
           'y': 1.15,
           'steps': 12,
           'waypoints': [1.21, 0.26],
           'closest_waypoints': 2,
           'is_offtrack': False,
           'is_crashed': False
           }


params7 = {'all_wheels_on_track': True,
           'track_width': 3,
           'distance_from_center': 1.4,
           'is_left_of_center': True,
           'heading': 0,
           'progress': 12.1,
           'steering_angle': -30,
           'speed': 5.33,
           'x': 3.12,
           'y': 1.15,
           'steps': 12,
           'waypoints': [1.21, 0.26],
           'closest_waypoints': 2,
           'is_offtrack': False,
           'is_crashed': False
           }

params8 = {'all_wheels_on_track': True,
           'track_width': 3,
           'distance_from_center': 1.4,
           'is_left_of_center': False,
           'heading': 0,
           'progress': 12.1,
           'steering_angle': 30,
           'speed': 5.33,
           'x': 3.12,
           'y': 1.15,
           'steps': 12,
           'waypoints': [1.21, 0.26],
           'closest_waypoints': 2,
           'is_offtrack': False,
           'is_crashed': False
           }


def get_test_params_from_file():
    file_name = 'C:\\Users\\PC\\Downloads\\0-iteration.csv'
    df = pd.read_csv(file_name, usecols=['episode', 'steps', 'X', 'Y', 'yaw', 'steer', 'throttle', 'action', 'reward', 'done',
                     'all_wheels_on_track', 'progress', 'closest_waypoint', 'track_len', 'tstamp', 'episode_status', 'pause_duration'])
    # row 0 -> step 1
    row = 5
    current_step = df.loc[row]
    print(current_step)
    params = {
        'steps': round(current_step['steps']),
        'x': current_step['X'],
        'y': current_step['Y'],
        'heading': current_step['yaw'],  # agent's yaw in degrees
        # agent's speed in meters per second (m/s)
        'speed': current_step['throttle'],
        'progress': current_step['progress'],
        'track_width': current_step['track_len'],
        'is_offtrack': current_step['episode_status'] == 'off_track',
        'steering_angle': current_step['steer'],
        'is_left_of_center': True,
        'distance_from_center': 0,
        'is_crashed': False
    }
    return params


def get_test_params_from_sim_trace_log():
    sim_trace_log = '575,24,-6.2329,-2.8798,-42.1327,30.00,2.80,6,26.8870,False,True,6.9454,126,46.12,1672.797,in_progress,0.00'

    # Split the line by ","
    data = sim_trace_log.split(",")
    current_step = pd.DataFrame([data], columns=['episode', 'steps', 'X', 'Y', 'yaw', 'steer', 'throttle', 'action', 'reward',
                                'done', 'all_wheels_on_track', 'progress', 'closest_waypoint', 'track_len', 'tstamp', 'episode_status', 'pause_duration'])

    print(current_step)
    params = {
        'steps': int(current_step['steps'].iloc[0]),
        'x': float(current_step['X'].iloc[0]),
        'y': float(current_step['Y'].iloc[0]),
        'heading': float(current_step['yaw'].iloc[0]),  # agent's yaw in degrees
        # agent's speed in meters per second (m/s)
        'speed': float(current_step['throttle'].iloc[0]),
        'progress': float(current_step['progress'].iloc[0]),
        'track_width': float(current_step['track_len'].iloc[0]),
        'is_offtrack': current_step['episode_status'].iloc[0] == 'off_track',
        'steering_angle': float(current_step['steer'].iloc[0]),
        'is_left_of_center': True,
        'distance_from_center': 0,
        'is_crashed': False
    }
    return params


# print ('Off Track, left, 0 steering: {}'.format(reward_function(offtrack)))
# print ('near middle Track, right, 10 steering: {}'.format(reward_function(params2)))
# print ('nearer middle Track, left, -10 steering: {}'.format(reward_function(params3)))
# print ('Middle Track: {}'.format(reward_function(params4)))
# print ('Turn sharp left when near left offtrack: {}'.format(reward_function(params5)))
# print ('Turn sharp right when near right offtrack: {}'.format(reward_function(params6)))
# print ('Turn sharp left when almost left offtrack: {}'.format(reward_function(params7)))
# print ('Turn sharp right when almost right offtrack: {}'.format(reward_function(params8)))


def get_test_params():
    params = {'all_wheels_on_track': True, 'x': 7.666686955033658, 'y': -2.607235358934886,
              'heading': 88.43765807467159, 'distance_from_center': 0.0017573151666297755,
              'projection_distance': 7.058494216287594, 'progress': 0.30350672421458336, 'steps': 2.0, 'speed': 3.9795,
              'steering_angle': -0.5551, 'track_width': 1.0667786098174583, 'track_length': 46.12337775575979,
              'waypoints': [], 'closest_waypoints': [23, 24], 'is_left_of_center': False, 'is_reversed': False,
              'closest_objects': [0, 0], 'objects_location': [], 'objects_left_of_center': [],
              'object_in_camera': False, 'objects_speed': [], 'objects_heading': [], 'objects_distance_from_center': [],
              'objects_distance': [], 'is_crashed': False, 'is_offtrack': False}
    # print(params)
    return params


def test_reward():

    # params = get_test_params_from_sim_trace_log()
    params = get_test_params()
    print(params)

    reward = reward_function(params)

    print("test_reward: {}".format(reward))

    assert reward > 0.0


def generate_action_space_metadata():
    import json
    import numpy as np
    action_spaces = []
    MAX_SPEED = 5
    MIN_SPEED = 1.5
    angle_sequence = list(range(-30, 30 + 5, 5))
    speed_sequence = np.arange(MIN_SPEED, MAX_SPEED + 0.5, 0.5)
    index = 0
    speed_map = {
        0: {
            "start": 2.0,
            "end": 8.0,
        },
        5: {
            "start": 2.0,
            "end": 4.0,
        },
        10: {
            "start": 2.0,
            "end": 4.0,
        },
        15: {
            "start": 1.5,
            "end": 3.5,
        },
        20: {
            "start": 1.5,
            "end": 3.5,
        },
        25: {
            "start": 1.5,
            "end": 3.0,
        },
        30: {
            "start": 1.5,
            "end": 3.0,
        }
    }
    for angle in angle_sequence:
        for speed in speed_sequence:
            action_space = {
                "steering_angle": angle,
                "speed": speed,
                "index": index
            }
            if speed_map.get(abs(angle)).get("start") <= speed <= speed_map.get(abs(angle)).get("end"):
                index += 1
                action_spaces.append(action_space)
    metadata_json = {
        "action_space": action_spaces,
        "sensor": ["FRONT_FACING_CAMERA"],
        "neural_network": "DEEP_CONVOLUTIONAL_NETWORK_SHALLOW",
        "training_algorithm": "clipped_ppo",
        "action_space_type": "discrete",
        "version": "5"
    }

    # Specify the file path where you want to save the JSON file
    filename = "model_metadata_{}_AS_max_{}_min_{}.json".format(index, MAX_SPEED, MIN_SPEED)
    filepath = "C:\\Users\\PC\\workspace\\deepracer\\deepracer\\model_metadata\\" + filename

    # Use json.dump() to write the data into the JSON file
    with open(filepath, 'w') as f:
        json.dump(metadata_json, f)

    print(f"Data has been saved to {filepath}")

def generate_action_space_metadata2():
    import json
    import numpy as np
    action_spaces = []
    angle_sequence = list(range(-30, 30 + 5, 5))
    speed_sequence = [1.5, 3]
    index = 0
    for angle in angle_sequence:
        for speed in speed_sequence:
            action_space = {
                "steering_angle": angle,
                "speed": speed,
                "index": index
            }
            action_spaces.append(action_space)
            index += 1
    metadata_json = {
        "action_space": action_spaces,
        "sensor": ["FRONT_FACING_CAMERA"],
        "neural_network": "DEEP_CONVOLUTIONAL_NETWORK_SHALLOW",
        "training_algorithm": "clipped_ppo",
        "action_space_type": "discrete",
        "version": "5"
    }

    # Specify the file path where you want to save the JSON file
    filename = "model_metadata_{}_linear_AS_1.5_3.json".format(index)
    filepath = "C:\\Users\\PC\\workspace\\deepracer\\deepracer\\model_metadata\\" + filename

    # Use json.dump() to write the data into the JSON file
    with open(filepath, 'w') as f:
        json.dump(metadata_json, f)

    print(f"Data has been saved to {filepath}")

def print_waypoints():

    array = [[6.81785, 2.89207, 4.0, 0.04971],
            [6.65272, 2.91452, 4.0, 0.04166],
            [6.50241, 2.93244, 4.0, 0.03784],
            [6.33688, 2.95081, 4.0, 0.04164],
            [6.03704, 2.9837, 4.0, 0.07541],
            [5.73721, 3.01667, 4.0, 0.07541],
            [5.43738, 3.04972, 4.0, 0.07541],
            [5.13756, 3.08285, 4.0, 0.07541],
            [4.83775, 3.11605, 4.0, 0.07541],
            [4.53794, 3.14924, 4.0, 0.07541],
            [4.23813, 3.18241, 4.0, 0.07541],
            [3.93841, 3.2152, 4.0, 0.07538],
            [3.63901, 3.24683, 4.0, 0.07527],
            [3.34018, 3.27652, 4.0, 0.07508],
            [3.04218, 3.30343, 4.0, 0.07481],
            [2.74526, 3.32674, 4.0, 0.07446],
            [2.44972, 3.34562, 4.0, 0.07404],
            [2.15583, 3.35926, 4.0, 0.07355],
            [1.86388, 3.3669, 4.0, 0.07301],
            [1.57433, 3.36728, 4.0, 0.07239],
            [1.28771, 3.35904, 3.72719, 0.07693],
            [1.00472, 3.34042, 3.72719, 0.07609],
            [0.72621, 3.30944, 3.72719, 0.07519],
            [0.45323, 3.26372, 3.72719, 0.07426],
            [0.18727, 3.20011, 3.72719, 0.07337],
            [-0.06923, 3.11357, 3.72719, 0.07263],
            [-0.31295, 2.99814, 4.0, 0.06742],
            [-0.54791, 2.86396, 4.0, 0.06764],
            [-0.77701, 2.71753, 4.0, 0.06797],
            [-1.00333, 2.56535, 4.0, 0.06818],
            [-1.23691, 2.41275, 4.0, 0.06975],
            [-1.47314, 2.26391, 4.0, 0.0698],
            [-1.71256, 2.12036, 4.0, 0.06979],
            [-1.95558, 1.9837, 4.0, 0.0697],
            [-2.20267, 1.8559, 4.0, 0.06955],
            [-2.45443, 1.73953, 4.0, 0.06934],
            [-2.71151, 1.63741, 4.0, 0.06915],
            [-2.97486, 1.5534, 4.0, 0.06911],
            [-3.24483, 1.49004, 4.0, 0.06933],
            [-3.51965, 1.44306, 4.0, 0.0697],
            [-3.79826, 1.40978, 4.0, 0.07015],
            [-4.07993, 1.38815, 4.0, 0.07062],
            [-4.36403, 1.37636, 4.0, 0.07109],
            [-4.65002, 1.37253, 4.0, 0.0715],
            [-4.93738, 1.37477, 4.0, 0.07184],
            [-5.22573, 1.3816, 4.0, 0.07211],
            [-5.50801, 1.3917, 3.57053, 0.07911],
            [-5.78852, 1.39663, 3.14156, 0.0893],
            [-6.06562, 1.39176, 3.14156, 0.08822],
            [-6.3377, 1.3728, 3.14156, 0.08682],
            [-6.6028, 1.33517, 3.14156, 0.08523],
            [-6.85855, 1.2742, 3.14156, 0.08369],
            [-7.10111, 1.1837, 3.14156, 0.08241],
            [-7.32482, 1.05732, 3.52014, 0.07299],
            [-7.53259, 0.90544, 3.26643, 0.07879],
            [-7.72388, 0.73098, 2.99873, 0.08634],
            [-7.8974, 0.53608, 2.66075, 0.09807],
            [-8.04967, 0.32165, 2.35909, 0.11148],
            [-8.17647, 0.09006, 2.08368, 0.12671],
            [-8.27229, -0.15474, 2.08368, 0.12617],
            [-8.33019, -0.4062, 2.08368, 0.12384],
            [-8.34431, -0.65461, 2.08368, 0.11941],
            [-8.30911, -0.88837, 2.08368, 0.11345],
            [-8.22184, -1.09536, 2.08368, 0.10781],
            [-8.08127, -1.26037, 2.23603, 0.09694],
            [-7.90314, -1.38332, 2.51581, 0.08603],
            [-7.69903, -1.46867, 2.78805, 0.07935],
            [-7.47551, -1.51907, 3.13774, 0.07302],
            [-7.23778, -1.53788, 3.4948, 0.06824],
            [-6.98954, -1.52773, 3.93769, 0.06309],
            [-6.73394, -1.49196, 3.71012, 0.06956],
            [-6.47343, -1.43427, 3.29804, 0.08091],
            [-6.21428, -1.35829, 2.92907, 0.0922],
            [-5.95834, -1.30327, 2.92907, 0.08937],
            [-5.70555, -1.27032, 2.92907, 0.08704],
            [-5.45609, -1.26088, 2.92907, 0.08523],
            [-5.21075, -1.27818, 2.92907, 0.08397],
            [-4.97131, -1.3282, 2.92907, 0.08351],
            [-4.74125, -1.4197, 3.46887, 0.07137],
            [-4.51948, -1.5419, 4.0, 0.0633],
            [-4.30381, -1.68494, 4.0, 0.0647],
            [-4.09119, -1.83808, 4.0, 0.06551],
            [-3.86599, -1.99828, 4.0, 0.06909],
            [-3.63852, -2.15631, 4.0, 0.06924],
            [-3.40772, -2.31086, 4.0, 0.06944],
            [-3.17271, -2.46047, 4.0, 0.06965],
            [-2.93269, -2.60342, 4.0, 0.06984],
            [-2.68695, -2.73763, 4.0, 0.07],
            [-2.4347, -2.86063, 4.0, 0.07016],
            [-2.17307, -2.96577, 4.0, 0.07049],
            [-1.90405, -3.05559, 4.0, 0.0709],
            [-1.62932, -3.13246, 4.0, 0.07132],
            [-1.35007, -3.1978, 4.0, 0.0717],
            [-1.06735, -3.25276, 4.0, 0.072],
            [-0.782, -3.29818, 4.0, 0.07223],
            [-0.49475, -3.33477, 4.0, 0.07239],
            [-0.20619, -3.36316, 4.0, 0.07249],
            [0.08323, -3.38379, 4.0, 0.07254],
            [0.37311, -3.3971, 4.0, 0.07255],
            [0.66315, -3.40351, 4.0, 0.07253],
            [0.95311, -3.40322, 4.0, 0.07249],
            [1.24276, -3.39633, 4.0, 0.07243],
            [1.53187, -3.3827, 4.0, 0.07236],
            [1.82015, -3.36188, 4.0, 0.07226],
            [2.10715, -3.33249, 4.0, 0.07213],
            [2.39227, -3.29319, 4.0, 0.07195],
            [2.67468, -3.24212, 4.0, 0.07175],
            [2.95317, -3.17667, 4.0, 0.07152],
            [3.22615, -3.09368, 4.0, 0.07133],
            [3.49134, -2.989, 4.0, 0.07127],
            [3.7497, -2.86733, 4.0, 0.07139],
            [4.00226, -2.73217, 4.0, 0.07161],
            [4.24974, -2.58582, 4.0, 0.07188],
            [4.49272, -2.42995, 4.0, 0.07217],
            [4.73163, -2.2658, 4.0, 0.07247],
            [4.9668, -2.09431, 4.0, 0.07276],
            [5.19849, -1.91623, 4.0, 0.07306],
            [5.42691, -1.73214, 4.0, 0.07334],
            [5.65227, -1.5426, 4.0, 0.07362],
            [5.87477, -1.34813, 4.0, 0.07388],
            [6.09461, -1.14926, 4.0, 0.07411],
            [6.31199, -0.94643, 4.0, 0.07433],
            [6.527, -0.73996, 4.0, 0.07452],
            [6.73966, -0.53004, 4.0, 0.0747],
            [6.94986, -0.31667, 4.0, 0.07488],
            [7.15708, -0.09943, 3.82296, 0.07853],
            [7.36075, 0.12204, 3.32532, 0.09048],
            [7.55989, 0.34831, 2.97096, 0.10145],
            [7.75296, 0.58004, 2.58827, 0.11654],
            [7.93175, 0.81611, 2.27708, 0.13005],
            [8.0842, 1.05542, 2.0, 0.14187],
            [8.20137, 1.29559, 2.0, 0.13362],
            [8.27842, 1.53346, 2.0, 0.12502],
            [8.31305, 1.76572, 2.0, 0.11741],
            [8.30027, 1.98795, 2.0, 0.1113],
            [8.23384, 2.19368, 2.0, 0.10809],
            [8.10225, 2.3699, 2.4777, 0.08876],
            [7.92766, 2.5196, 2.84138, 0.08094],
            [7.7188, 2.64276, 3.2385, 0.07487],
            [7.48516, 2.73902, 3.80246, 0.06645],
            [7.24296, 2.81019, 4.0, 0.06311],
            [7.01403, 2.85965, 4.0, 0.05855]]
    
    new_array = []
    for i in range(len(array)):
        new_array.append([array[i][0], array[i][1]])

    print('[')
    index = 0
    for row in new_array:
        # Print each element in the row, separated by a space
        if index != len(new_array)-1:
            print('{},'.format(row))
        else:
            print('{}'.format(row))
        index += 1
    print(']')
    return array


if __name__ == '__main__':
    # test_reward()
    # generate_action_space_metadata2()
    print_waypoints()
