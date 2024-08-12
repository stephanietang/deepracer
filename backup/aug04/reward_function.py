# Fix the bug of version 3, adding more loggings
import math


class Reward:
    def __init__(self, verbose=False, debug=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose
        self.debug = debug

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
        def get_projected_time(first_index, closest_index, step_count):

            times_list = [row[3] for row in racing_track]

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            if self.verbose:
                print("first_index: %i" % first_index)
            
            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(
                first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum(
                [times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time /
                                  current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        def get_distance_reward():
            ## Reward if car goes close to optimal racing line ##
            DISTANCE_MULTIPLE = 1
            dist = dist_to_racing_line(
                optimals[0:2], optimals_second[0:2], [x, y])
            distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
            final_distance_reward = distance_reward * DISTANCE_MULTIPLE
            return dist, final_distance_reward

        def get_direction_reward():
            # Zero reward if obviously wrong direction (e.g. spin)
            direction_diff = racing_direction_diff(
                optimals[0:2], optimals_second[0:2], [x, y], heading)

            STEERING_ANGLE_THRESHOLD = 30
            if direction_diff > STEERING_ANGLE_THRESHOLD:
                reward = 1e-3
            else:
                reward = 1.0 - direction_diff / STEERING_ANGLE_THRESHOLD
            return direction_diff, reward

        def get_speed_reward():
            ## Reward if speed is close to optimal speed ##
            SPEED_MULTIPLE = 1  # change the multiple to emphasize on speed
            SPEED_DIFF_NO_REWARD = 1
            speed_diff = abs(optimals[2]-speed)
            if speed_diff <= SPEED_DIFF_NO_REWARD:
                # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
                # so, we do not punish small deviations from optimal speed
                speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
            else:
                speed_reward = 1e-3
            final_speed_reward = speed_reward * SPEED_MULTIPLE
            return speed_diff, final_speed_reward

        def get_step_reward():
            # Reward if less steps
            REWARD_PER_STEP_FOR_FASTEST_TIME = 1
            FASTEST_TIME = 15  # seconds (best time of 1st place on the track)
            STANDARD_TIME = 22
            projected_time = get_projected_time(
                self.first_racingpoint_index, closest_index, steps)
            try:
                steps_prediction = projected_time * 15 + 1
                reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                               (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
                steps_reward = min(
                    REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
            except:
                steps_reward = 0
            return projected_time, steps_reward

        def get_finish_reward():
            ## Incentive for finishing the lap in less steps ##
            STANDARD_TIME = 25  # seconds (time that is easily done by model)
            if progress == 100:
                finish_reward = max(1e-3, -1 * (steps-STANDARD_TIME*15))
            else:
                finish_reward = 0
            return finish_reward
        
        def remove_keys(d, keys):
            return {k: v for k, v in d.items() if k not in keys}

        #################### RACING LINE ######################

        # Optimal racing line
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[6.83743, 2.89489, 4.0, 0.06136],
                        [6.58874, 2.92402, 4.0, 0.0626],
                        [6.3388, 2.95137, 4.0, 0.06286],
                        [6.08885, 2.97878, 4.0, 0.06286],
                        [5.83892, 3.00624, 4.0, 0.06286],
                        [5.58898, 3.03376, 4.0, 0.06286],
                        [5.33906, 3.06133, 4.0, 0.06286],
                        [5.08914, 3.08896, 4.0, 0.06286],
                        [4.83923, 3.11664, 4.0, 0.06286],
                        [4.58931, 3.1443, 4.0, 0.06286],
                        [4.33939, 3.17195, 4.0, 0.06286],
                        [4.08948, 3.19958, 4.0, 0.06286],
                        [3.83955, 3.22719, 4.0, 0.06286],
                        [3.58963, 3.25479, 4.0, 0.06286],
                        [3.33971, 3.28237, 4.0, 0.06286],
                        [3.08978, 3.30994, 4.0, 0.06286],
                        [2.84023, 3.33626, 4.0, 0.06273],
                        [2.59141, 3.36005, 4.0, 0.06249],
                        [2.34371, 3.38009, 4.0, 0.06213],
                        [2.09752, 3.39519, 4.0, 0.06166],
                        [1.85324, 3.40424, 4.0, 0.06111],
                        [1.61126, 3.40621, 4.0, 0.0605],
                        [1.37199, 3.4001, 4.0, 0.05984],
                        [1.13582, 3.385, 4.0, 0.05916],
                        [0.90317, 3.36007, 3.59903, 0.06501],
                        [0.67446, 3.3245, 3.59903, 0.06431],
                        [0.45015, 3.27746, 3.59903, 0.06368],
                        [0.23073, 3.21811, 3.59903, 0.06316],
                        [0.01679, 3.14555, 3.59903, 0.06277],
                        [-0.191, 3.05877, 3.59903, 0.06257],
                        [-0.39066, 2.95425, 4.0, 0.05634],
                        [-0.58394, 2.83648, 4.0, 0.05658],
                        [-0.77283, 2.70983, 4.0, 0.05686],
                        [-0.95947, 2.57865, 4.0, 0.05703],
                        [-1.15148, 2.4475, 4.0, 0.05813],
                        [-1.34583, 2.31979, 4.0, 0.05814],
                        [-1.54285, 2.19649, 4.0, 0.05811],
                        [-1.74285, 2.0786, 4.0, 0.05804],
                        [-1.94606, 1.96714, 4.0, 0.05794],
                        [-2.15271, 1.8631, 4.0, 0.05784],
                        [-2.36301, 1.76756, 4.0, 0.05775],
                        [-2.57714, 1.68158, 4.0, 0.05769],
                        [-2.79525, 1.60629, 4.0, 0.05769],
                        [-3.01752, 1.54296, 4.0, 0.05778],
                        [-3.24325, 1.49035, 4.0, 0.05795],
                        [-3.47206, 1.44775, 4.0, 0.05818],
                        [-3.70357, 1.41451, 4.0, 0.05847],
                        [-3.93748, 1.38994, 4.0, 0.0588],
                        [-4.17351, 1.37341, 4.0, 0.05915],
                        [-4.41138, 1.36423, 4.0, 0.05951],
                        [-4.65084, 1.36171, 4.0, 0.05987],
                        [-4.89166, 1.36514, 4.0, 0.06021],
                        [-5.13362, 1.37382, 4.0, 0.06053],
                        [-5.36974, 1.38663, 4.0, 0.05912],
                        [-5.60429, 1.39436, 3.98472, 0.05889],
                        [-5.83664, 1.39517, 3.75004, 0.06196],
                        [-6.06616, 1.38732, 3.54854, 0.06472],
                        [-6.29221, 1.36923, 3.45839, 0.06557],
                        [-6.51416, 1.33947, 3.34665, 0.06691],
                        [-6.73129, 1.2967, 3.24024, 0.0683],
                        [-6.94285, 1.23961, 3.13768, 0.06984],
                        [-7.14792, 1.16685, 3.04355, 0.07149],
                        [-7.34542, 1.07702, 2.9444, 0.07369],
                        [-7.53442, 0.96945, 2.8514, 0.07627],
                        [-7.71361, 0.84299, 2.76225, 0.0794],
                        [-7.88117, 0.69618, 2.674, 0.08331],
                        [-8.03461, 0.52718, 2.54741, 0.0896],
                        [-8.17027, 0.33401, 2.42689, 0.09727],
                        [-8.28225, 0.11564, 2.32076, 0.10574],
                        [-8.35909, -0.11259, 2.21933, 0.10851],
                        [-8.39782, -0.33515, 2.10516, 0.10731],
                        [-8.40194, -0.54772, 2.10516, 0.10099],
                        [-8.37384, -0.74744, 2.10516, 0.09581],
                        [-8.31546, -0.93164, 2.10516, 0.09179],
                        [-8.22845, -1.0975, 2.10516, 0.08897],
                        [-8.1141, -1.24152, 2.10516, 0.08736],
                        [-7.97321, -1.35853, 2.23012, 0.08212],
                        [-7.81159, -1.4486, 2.37791, 0.07781],
                        [-7.63356, -1.51229, 2.55878, 0.07389],
                        [-7.44254, -1.55048, 2.78134, 0.07004],
                        [-7.24137, -1.56452, 3.0707, 0.06567],
                        [-7.0325, -1.55639, 3.47144, 0.06021],
                        [-6.81814, -1.52907, 3.86895, 0.05585],
                        [-6.60017, -1.48658, 3.35129, 0.06627],
                        [-6.38018, -1.43404, 2.78815, 0.08112],
                        [-6.16973, -1.38001, 2.78815, 0.07793],
                        [-5.95964, -1.33165, 2.78815, 0.07732],
                        [-5.75027, -1.29391, 2.78815, 0.0763],
                        [-5.54211, -1.27133, 2.78815, 0.0751],
                        [-5.33585, -1.26806, 2.78815, 0.07399],
                        [-5.13311, -1.29162, 2.79508, 0.07302],
                        [-4.9348, -1.34028, 2.79508, 0.07306],
                        [-4.74238, -1.41527, 3.12948, 0.06599],
                        [-4.55572, -1.51189, 3.6168, 0.05811],
                        [-4.37398, -1.62544, 4.0, 0.05357],
                        [-4.19584, -1.75084, 4.0, 0.05446],
                        [-4.01963, -1.88243, 4.0, 0.05498],
                        [-3.83206, -2.01909, 4.0, 0.05802],
                        [-3.64205, -2.15306, 4.0, 0.05812],
                        [-3.44928, -2.28367, 4.0, 0.05821],
                        [-3.25345, -2.4102, 4.0, 0.05829],
                        [-3.05431, -2.53193, 4.0, 0.05835],
                        [-2.85166, -2.64821, 4.0, 0.05841],
                        [-2.64528, -2.75831, 4.0, 0.05848],
                        [-2.43499, -2.86152, 4.0, 0.05856],
                        [-2.22053, -2.95707, 4.0, 0.05869],
                        [-2.00212, -3.04503, 4.0, 0.05886],
                        [-1.77992, -3.1255, 4.0, 0.05908],
                        [-1.55405, -3.19852, 4.0, 0.05935],
                        [-1.32462, -3.26414, 4.0, 0.05966],
                        [-1.09178, -3.32245, 4.0, 0.06001],
                        [-0.85566, -3.37358, 4.0, 0.0604],
                        [-0.61646, -3.41774, 4.0, 0.06081],
                        [-0.37438, -3.45516, 4.0, 0.06124],
                        [-0.12962, -3.4861, 4.0, 0.06168],
                        [0.11759, -3.51082, 4.0, 0.06211],
                        [0.36694, -3.52958, 4.0, 0.06251],
                        [0.61319, -3.54244, 4.0, 0.06165],
                        [0.84617, -3.54976, 4.0, 0.05827],
                        [1.07428, -3.55184, 4.0, 0.05703],
                        [1.29974, -3.5484, 4.0, 0.05637],
                        [1.52367, -3.53906, 4.0, 0.05603],
                        [1.74674, -3.5233, 4.0, 0.05591],
                        [1.96934, -3.50066, 4.0, 0.05594],
                        [2.19164, -3.47066, 4.0, 0.05608],
                        [2.41367, -3.43284, 4.0, 0.05631],
                        [2.63536, -3.38678, 4.0, 0.05661],
                        [2.8565, -3.33207, 4.0, 0.05695],
                        [3.07685, -3.2684, 4.0, 0.05734],
                        [3.29606, -3.19551, 4.0, 0.05775],
                        [3.5138, -3.11327, 4.0, 0.05819],
                        [3.72968, -3.02157, 4.0, 0.05864],
                        [3.94332, -2.92052, 4.0, 0.05909],
                        [4.15442, -2.81032, 4.0, 0.05953],
                        [4.36266, -2.69127, 4.0, 0.05997],
                        [4.56782, -2.56373, 4.0, 0.06039],
                        [4.76971, -2.42815, 4.0, 0.0608],
                        [4.96821, -2.28505, 4.0, 0.06118],
                        [5.16323, -2.1349, 4.0, 0.06153],
                        [5.35473, -1.97826, 4.0, 0.06185],
                        [5.54274, -1.81572, 4.0, 0.06213],
                        [5.72734, -1.64791, 4.0, 0.06237],
                        [5.90864, -1.47545, 4.0, 0.06256],
                        [6.08685, -1.29899, 4.0, 0.0627],
                        [6.26219, -1.11917, 4.0, 0.06279],
                        [6.43494, -0.93659, 4.0, 0.06284],
                        [6.60542, -0.75179, 4.0, 0.06286],
                        [6.77401, -0.56524, 4.0, 0.06286],
                        [6.94111, -0.37735, 4.0, 0.06286],
                        [7.10682, -0.18825, 4.0, 0.06286],
                        [7.27117, 0.00205, 3.58487, 0.07014],
                        [7.43416, 0.19351, 3.0824, 0.08157],
                        [7.59578, 0.38613, 2.7453, 0.09159],
                        [7.75597, 0.57994, 2.4905, 0.10096],
                        [7.90884, 0.77609, 2.29235, 0.10849],
                        [8.04492, 0.97473, 2.13319, 0.11288],
                        [8.15667, 1.17475, 2.0, 0.11456],
                        [8.23931, 1.37421, 2.0, 0.10795],
                        [8.29041, 1.57085, 2.0, 0.10159],
                        [8.3087, 1.76232, 2.0, 0.09617],
                        [8.29332, 1.94603, 2.0, 0.09218],
                        [8.24301, 2.11885, 2.0, 0.09],
                        [8.15538, 2.27627, 2.20549, 0.08169],
                        [8.03731, 2.41791, 2.36072, 0.07811],
                        [7.89156, 2.54258, 2.54867, 0.07525],
                        [7.72029, 2.64914, 2.79001, 0.0723],
                        [7.52582, 2.73685, 3.10654, 0.06867],
                        [7.31107, 2.80573, 3.55721, 0.0634],
                        [7.07997, 2.85726, 4.0, 0.0592]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        if self.verbose:
            params = remove_keys(params, ['waypoints', 'closest_objects', 'objects_location', 'objects_left_of_center',
                                        'object_in_camera', 'objects_speed', 'objects_heading', 'objects_distance_from_center', 'objects_distance', 
                                        'projection_distance'])
            print("params: ", params)

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
        if self.debug:
            self.first_racingpoint_index = 0  # this is just for testing purposes

        if steps == 2.0:
            # when steps == 1, status is prepare, thus reset the first_racingpoint_index when steps = 2
            print('set first_racingpoint_index to %i' % closest_index)
            self.first_racingpoint_index = closest_index
        else:
            # print('first_racingpoint_index: %i' % self.first_racingpoint_index)
            pass

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        STANDARD_REWARD = 1
        reward = STANDARD_REWARD

        dist, distance_reward = get_distance_reward()
        reward += distance_reward

        # speed_diff, speed_reward = get_speed_reward()
        # reward += speed_reward

        # projected_time, steps_reward = get_step_reward()
        # reward += steps_reward

        # direction_diff, direction_reward = get_direction_reward()
        # reward += direction_reward

        # finish_reward = get_finish_reward()
        # reward += finish_reward

        ## Zero reward if off track ##
        if is_offtrack:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            # printStr = ("REWARD: {:3.4f}, DIS_REW: {:3.4f}, SPD_REW: {:3.4f}, DIR_REW: {:3.4f}, "
            #             "FIN_REW: {:3.4f}, ACT_SPD: {:3.4f}, EXP_SPD: {:3.4f}, SPD_DIFF: {:3.4f}, "
            #             "CLOSET_INDEX: {}, DIST: {:3.4f}, DIR_DIFF: {:3.4f}, PROJ_TIME: {:3.4f}").format(reward,
            #                                                                                              distance_reward, speed_reward,
            #                                                                                              direction_reward, finish_reward, speed, optimals[
            #                                                                                                  2], speed_diff,
            #                                                                                              closest_index, dist, direction_diff, projected_time)
            printStr = ("REWARD: {:3.4f}, DIS_REW: {:3.4f}, DIST: {:3.4f}, CLOSET_INDEX: {}").format(
                reward, distance_reward, dist, closest_index)
            print(printStr)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


# add parameter verbose=True to get noisy output for testing
reward_object = Reward(verbose=True, debug=False)


def reward_function(params):
    return reward_object.reward_function(params)
