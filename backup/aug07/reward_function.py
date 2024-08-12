# Fix the bug of version 3, adding more loggings
import math
import numpy as np


class Reward:
    def __init__(self, verbose=False, debug=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose
        self.debug = debug

    def reward_function(self, params):

        FUTURE_STEP = 6
        SPEED_THRESHOLD_SLOW = 1.5  # m/s
        SPEED_THRESHOLD_FAST = 2.8    # m/s
        TURN_THRESHOLD_SPEED = 6    # degrees
        STEERING_THRESHOLD = 11     # degrees

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

        def identify_corner(waypoints, closest_waypoints, future_step):

            # Identify next waypoint and a further waypoint
            point_prev = waypoints[closest_waypoints[0]]
            point_next = waypoints[closest_waypoints[1]]
            point_future = waypoints[min(len(waypoints) - 1,
                                         closest_waypoints[1] + future_step)]

            # Calculate headings to waypoints
            heading_current = math.degrees(math.atan2(point_prev[1]-point_next[1],
                                                      point_prev[0] - point_next[0]))
            heading_future = math.degrees(math.atan2(point_prev[1] - point_future[1],
                                                     point_prev[0] - point_future[0]))

            # Calculate the difference between the headings
            diff_heading = abs(heading_current - heading_future)

            # Check we didn't choose the reflex angle
            if diff_heading > 180:
                diff_heading = 360 - diff_heading

            # Calculate distance to further waypoint
            dist_future = np.linalg.norm([point_next[0] - point_future[0],
                                          point_next[1] - point_future[1]])

            return diff_heading, dist_future

        def select_speed(waypoints, closest_waypoints, future_step):

            # Identify if a corner is in the future
            diff_heading, dist_future = identify_corner(waypoints,
                                                        closest_waypoints,
                                                        future_step)

            if diff_heading < TURN_THRESHOLD_SPEED:
                # If there's no corner encourage going faster
                go_fast = True
            else:
                # If there is a corner encourage slowing down
                go_fast = False

            return go_fast

        def get_speed_reward2():
            go_fast = select_speed(waypoints, closest_waypoints, FUTURE_STEP)
            if go_fast and speed > SPEED_THRESHOLD_FAST and abs(steering_angle) < STEERING_THRESHOLD:
                return 1
            elif not go_fast and speed <= SPEED_THRESHOLD_SLOW:
                return 1
            else:
                return 1e-3

    #################### RACING LINE ######################

        # Optimal racing line
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[6.81785, 2.89207, 4.0, 0.04971],
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

        ################## INPUT PARAMETERS ###################

        is_offtrack = params['is_offtrack']
        x = params['x']
        y = params['y']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        track_width = params['track_width']
        steering_angle = params['steering_angle']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        # steering_angle = params['steering_angle']
        # waypoints = params['waypoints']
        # closest_waypoints = params['closest_waypoints']
        # is_offtrack = params['is_offtrack']
        # distance_from_center = params['distance_from_center']
        # is_left_of_center = params['is_left_of_center']

        # Read all input parameters
        if self.verbose:
            params = remove_keys(params, ['waypoints', 'closest_objects', 'objects_location', 'objects_left_of_center',
                                          'object_in_camera', 'objects_speed', 'objects_heading', 'objects_distance_from_center', 'objects_distance',
                                          'projection_distance'])
            print("params: ", params)

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

        speed_reward = get_speed_reward2()

        # speed_diff, speed_reward = get_speed_reward()
        # reward += speed_reward

        # projected_time, steps_reward = get_step_reward()
        # reward += steps_reward

        direction_diff, direction_reward = get_direction_reward()
        reward += direction_reward

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
            printStr = ("REWARD: {:3.4f}, DIST_REW: {:3.4f}, DIST: {:3.4f}, DIR_REW: {:3.4f}, DIR_DIFF: {:3.4f}, SPEED_REW: {:3.4f}, CLOSET_INDEX: {}").format(
                reward, distance_reward, dist, direction_reward, direction_diff, speed_reward, closest_index)
            print(printStr)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


# add parameter verbose=True to get noisy output for testing
reward_object = Reward(verbose=True, debug=False)


def reward_function(params):
    return reward_object.reward_function(params)
