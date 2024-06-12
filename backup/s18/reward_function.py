import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[1.63496, -3.96, 4.0, 0.07546],
                        [1.93681, -3.96, 4.0, 0.07546],
                        [2.23865, -3.96, 4.0, 0.07546],
                        [2.54049, -3.96, 4.0, 0.07546],
                        [2.84233, -3.96, 4.0, 0.07546],
                        [3.14417, -3.96, 4.0, 0.07546],
                        [3.44601, -3.96, 4.0, 0.07546],
                        [3.74785, -3.96, 4.0, 0.07546],
                        [4.04969, -3.96, 4.0, 0.07546],
                        [4.35153, -3.96, 4.0, 0.07546],
                        [4.65337, -3.96, 4.0, 0.07546],
                        [4.95521, -3.96, 4.0, 0.07546],
                        [5.25705, -3.96, 3.14174, 0.09607],
                        [5.55889, -3.96, 2.77481, 0.10878],
                        [5.86073, -3.96, 2.72933, 0.11059],
                        [6.16257, -3.96, 2.72933, 0.11059],
                        [6.46441, -3.96, 2.72933, 0.11059],
                        [6.75235, -3.92906, 2.72933, 0.1061],
                        [7.00992, -3.84285, 2.72933, 0.09952],
                        [7.22618, -3.69958, 2.72933, 0.09505],
                        [7.39634, -3.50633, 2.92357, 0.08807],
                        [7.52073, -3.27317, 3.30222, 0.08003],
                        [7.60277, -3.00994, 3.87101, 0.07123],
                        [7.64847, -2.72581, 3.58367, 0.0803],
                        [7.66645, -2.4292, 3.14978, 0.09434],
                        [7.66792, -2.1276, 3.09274, 0.09752],
                        [7.66521, -1.82577, 3.09274, 0.0976],
                        [7.66012, -1.52398, 3.09274, 0.09759],
                        [7.63015, -1.23162, 3.09274, 0.09503],
                        [7.55458, -0.96073, 3.09274, 0.09093],
                        [7.4279, -0.71971, 3.09274, 0.08804],
                        [7.25375, -0.51271, 3.20826, 0.08432],
                        [7.03816, -0.34185, 3.50329, 0.07852],
                        [6.78917, -0.20664, 3.9593, 0.07156],
                        [6.51575, -0.10425, 4.0, 0.07299],
                        [6.22698, -0.0293, 4.0, 0.07458],
                        [5.93078, 0.02574, 4.0, 0.07532],
                        [5.63202, 0.06869, 4.0, 0.07546],
                        [5.33234, 0.10465, 4.0, 0.07546],
                        [5.03208, 0.13538, 4.0, 0.07546],
                        [4.73147, 0.16249, 4.0, 0.07546],
                        [4.43091, 0.18743, 4.0, 0.0754],
                        [4.13325, 0.22309, 4.0, 0.07495],
                        [3.84573, 0.28743, 4.0, 0.07366],
                        [3.57354, 0.38722, 4.0, 0.07248],
                        [3.31873, 0.52167, 4.0, 0.07203],
                        [3.08123, 0.6869, 4.0, 0.07233],
                        [2.85953, 0.87771, 4.0, 0.07313],
                        [2.65011, 1.08741, 4.0, 0.07409],
                        [2.44781, 1.30859, 4.0, 0.07494],
                        [2.24127, 1.52829, 3.14056, 0.09602],
                        [2.02714, 1.74056, 2.65092, 0.11374],
                        [1.80197, 1.94114, 2.5, 0.12062],
                        [1.56348, 2.12523, 2.5, 0.12051],
                        [1.30985, 2.28794, 2.5, 0.12053],
                        [1.04016, 2.42274, 2.5, 0.1206],
                        [0.75603, 2.49073, 2.5, 0.11686],
                        [0.48364, 2.47132, 2.5, 0.10923],
                        [0.24384, 2.37112, 2.53991, 0.10232],
                        [0.04666, 2.20461, 2.74585, 0.09399],
                        [-0.1045, 1.98628, 3.12292, 0.08503],
                        [-0.21114, 1.72978, 3.01318, 0.09219],
                        [-0.27956, 1.448, 2.82236, 0.10274],
                        [-0.31998, 1.15219, 2.82236, 0.10579],
                        [-0.3447, 0.85136, 2.82236, 0.10694],
                        [-0.3687, 0.55185, 2.82236, 0.10646],
                        [-0.43398, 0.27601, 2.82236, 0.10043],
                        [-0.55311, 0.03714, 2.82236, 0.09458],
                        [-0.72239, -0.15847, 2.8804, 0.08981],
                        [-0.93411, -0.30958, 3.10848, 0.08368],
                        [-1.1804, -0.41723, 3.50799, 0.07662],
                        [-1.45321, -0.48472, 4.0, 0.07026],
                        [-1.74424, -0.51751, 4.0, 0.07322],
                        [-2.04449, -0.52325, 3.23568, 0.09281],
                        [-2.34607, -0.51111, 2.86295, 0.10542],
                        [-2.64665, -0.48398, 2.78314, 0.10844],
                        [-2.94496, -0.43829, 2.78314, 0.10843],
                        [-3.23856, -0.36864, 2.78314, 0.10842],
                        [-3.51872, -0.26022, 2.78314, 0.10794],
                        [-3.76435, -0.09751, 2.78314, 0.10586],
                        [-3.95217, 0.11818, 2.78314, 0.10276],
                        [-4.07121, 0.3721, 2.92439, 0.0959],
                        [-4.12397, 0.64775, 3.27273, 0.08575],
                        [-4.1207, 0.93292, 3.88058, 0.07349],
                        [-4.07529, 1.22046, 4.0, 0.07277],
                        [-4.00125, 1.50714, 3.1971, 0.09261],
                        [-3.90963, 1.79452, 2.67432, 0.11279],
                        [-3.82618, 2.08431, 2.514, 0.11995],
                        [-3.75459, 2.37727, 2.514, 0.11996],
                        [-3.69886, 2.67379, 2.514, 0.12001],
                        [-3.67101, 2.97237, 2.514, 0.11928],
                        [-3.7065, 3.25587, 2.514, 0.11365],
                        [-3.81503, 3.49987, 2.514, 0.10623],
                        [-3.98701, 3.69053, 2.53491, 0.10129],
                        [-4.20868, 3.82115, 2.67052, 0.09634],
                        [-4.46629, 3.88788, 2.90288, 0.09167],
                        [-4.74532, 3.89, 3.23783, 0.08618],
                        [-5.03071, 3.83249, 3.69454, 0.0788],
                        [-5.30991, 3.72651, 4.0, 0.07466],
                        [-5.57682, 3.58659, 4.0, 0.07534],
                        [-5.83218, 3.42573, 4.0, 0.07545],
                        [-6.07838, 3.25115, 3.92445, 0.07691],
                        [-6.31619, 3.0653, 3.21411, 0.0939],
                        [-6.54649, 2.87027, 3.16495, 0.09535],
                        [-6.77039, 2.66796, 3.16495, 0.09535],
                        [-6.98907, 2.46002, 3.16495, 0.09534],
                        [-7.19066, 2.25692, 3.16495, 0.09042],
                        [-7.30926, 2.10531, 3.16495, 0.06082],
                        [-7.41162, 1.92535, 3.16495, 0.06541],
                        [-7.49198, 1.70512, 3.56169, 0.06582],
                        [-7.54374, 1.45367, 4.0, 0.06418],
                        [-7.57003, 1.1831, 4.0, 0.06796],
                        [-7.57869, 0.90222, 4.0, 0.07025],
                        [-7.57686, 0.60353, 4.0, 0.07467],
                        [-7.58138, 0.30497, 4.0, 0.07465],
                        [-7.59021, 0.0065, 4.0, 0.07465],
                        [-7.59984, -0.29204, 4.0, 0.07467],
                        [-7.59386, -0.59025, 4.0, 0.07457],
                        [-7.56688, -0.88731, 4.0, 0.07457],
                        [-7.51642, -1.18192, 4.0, 0.07473],
                        [-7.44211, -1.47246, 4.0, 0.07497],
                        [-7.34477, -1.7571, 4.0, 0.07521],
                        [-7.22558, -2.03395, 4.0, 0.07535],
                        [-7.0858, -2.30131, 4.0, 0.07542],
                        [-6.92629, -2.55748, 4.0, 0.07544],
                        [-6.74793, -2.8009, 4.0, 0.07544],
                        [-6.55058, -3.02919, 4.0, 0.07544],
                        [-6.33423, -3.2395, 4.0, 0.07543],
                        [-6.09879, -3.42805, 4.0, 0.07541],
                        [-5.84417, -3.58909, 4.0, 0.07532],
                        [-5.57308, -3.71891, 4.0, 0.07514],
                        [-5.28954, -3.81642, 4.0, 0.07496],
                        [-4.99768, -3.88399, 4.0, 0.0749],
                        [-4.70074, -3.92614, 4.0, 0.07498],
                        [-4.401, -3.9487, 4.0, 0.07515],
                        [-4.09987, -3.95794, 4.0, 0.07532],
                        [-3.79816, -3.95989, 4.0, 0.07543],
                        [-3.49632, -3.96006, 4.0, 0.07546],
                        [-3.19448, -3.96008, 4.0, 0.07546],
                        [-2.89264, -3.96002, 4.0, 0.07546],
                        [-2.5908, -3.95999, 4.0, 0.07546],
                        [-2.28896, -3.95998, 4.0, 0.07546],
                        [-1.98712, -3.95999, 4.0, 0.07546],
                        [-1.68528, -3.96, 4.0, 0.07546],
                        [-1.38344, -3.96, 4.0, 0.07546],
                        [-1.0816, -3.96, 4.0, 0.07546],
                        [-0.77976, -3.96, 4.0, 0.07546],
                        [-0.47792, -3.96, 4.0, 0.07546],
                        [-0.17608, -3.96, 4.0, 0.07546],
                        [0.12576, -3.96, 4.0, 0.07546],
                        [0.4276, -3.96, 4.0, 0.07546],
                        [0.72944, -3.96, 4.0, 0.07546],
                        [1.03128, -3.96, 4.0, 0.07546],
                        [1.33312, -3.96, 4.0, 0.07546]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        is_offtrack = params['is_offtrack']
        x = params['x']
        y = params['y']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        track_width = params['track_width']
        # steering_angle = params['steering_angle']
        # waypoints = params['waypoints']
        # closest_waypoints = params['closest_waypoints']
        # is_offtrack = params['is_offtrack']
        # distance_from_center = params['distance_from_center']
        # is_left_of_center = params['is_left_of_center']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 27  # seconds (time that is easily done by model)
        FASTEST_TIME = 15  # seconds (best time of 1st place on the track)
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if is_offtrack == True:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward(True) # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)


def get_test_params():
    return {
        'x': 7.1874936,
        'y': -0.545777384,
        'steps': 74,
        'speed': 1.8, # agent's speed in meters per second (m/s)
        'heading': 132.0499605, # agent's yaw in degrees
        'progress': 20.08587563,
        'track_width': 46.07287348,
        'is_offtrack': False,
    }


def test_reward():
    params = get_test_params()

    reward = reward_function(params)

    print("test_reward: {}".format(reward))

    assert reward > 0.0


if __name__ == '__main__':
    test_reward()
    print('complete')
