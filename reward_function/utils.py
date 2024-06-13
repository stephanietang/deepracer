import pandas as pd

def reward_function(params):
    
    #Actual reward function go here
    from reward_function_progress_velocity import reward_function
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
    df = pd.read_csv(file_name, usecols=['episode', 'steps', 'X', 'Y', 'yaw', 'steer', 'throttle', 'action', 'reward', 'done', 'all_wheels_on_track', 'progress', 'closest_waypoint', 'track_len', 'tstamp', 'episode_status', 'pause_duration'])
    # row 0 -> step 1
    row = 5
    current_step = df.loc[row]
    print(current_step)
    params = {
        'steps': round(current_step['steps']),
        'x': current_step['X'],
        'y': current_step['Y'],
        'heading': current_step['yaw'], # agent's yaw in degrees
        'speed': current_step['throttle'], # agent's speed in meters per second (m/s)
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
    current_step = pd.DataFrame([data], columns=['episode', 'steps', 'X', 'Y', 'yaw', 'steer', 'throttle', 'action', 'reward', 'done', 'all_wheels_on_track', 'progress', 'closest_waypoint', 'track_len', 'tstamp', 'episode_status', 'pause_duration'])

    print(current_step)
    params = {
        'steps': int(current_step['steps']),
        'x': float(current_step['X']),
        'y': float(current_step['Y']),
        'heading': float(current_step['yaw']), # agent's yaw in degrees
        'speed': float(current_step['throttle']), # agent's speed in meters per second (m/s)
        'progress': float(current_step['progress']),
        'track_width': float(current_step['track_len']),
        'is_offtrack': current_step['episode_status'] == 'off_track',
        'steering_angle': float(current_step['steer']),
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
    #print(params)
    return params


def test_reward():

    params = get_test_params_from_sim_trace_log()

    reward = reward_function(params)

    print("test_reward: {}".format(reward))

    assert reward > 0.0


if __name__ == '__main__':
    test_reward()