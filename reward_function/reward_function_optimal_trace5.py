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

        #################### RACING LINE ######################

        # Optimal racing line
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
                        [4.65337, -3.96, 3.74576, 0.08058],
                        [4.95521, -3.96, 3.29753, 0.09154],
                        [5.25705, -3.96, 2.97377, 0.1015],
                        [5.55642, -3.95608, 2.72728, 0.10978],
                        [5.84812, -3.94015, 2.52295, 0.11579],
                        [6.12753, -3.90577, 2.52295, 0.11158],
                        [6.39091, -3.84844, 2.52295, 0.10684],
                        [6.63525, -3.76534, 2.52295, 0.10229],
                        [6.85783, -3.65452, 2.52295, 0.09855],
                        [7.05569, -3.51432, 2.52295, 0.09612],
                        [7.2245, -3.34246, 2.76574, 0.0871],
                        [7.3678, -3.14516, 2.88085, 0.08465],
                        [7.48537, -2.92411, 2.94427, 0.08503],
                        [7.57491, -2.67948, 2.97386, 0.0876],
                        [7.63188, -2.41142, 2.99993, 0.09135],
                        [7.65011, -2.12661, 2.89714, 0.09851],
                        [7.62686, -1.84376, 2.89714, 0.09796],
                        [7.5663, -1.57667, 2.89714, 0.09453],
                        [7.47365, -1.3286, 2.89714, 0.0914],
                        [7.35255, -1.10009, 2.89714, 0.08926],
                        [7.20538, -0.8914, 2.89714, 0.08814],
                        [7.03041, -0.70601, 3.10288, 0.08216],
                        [6.832, -0.5423, 3.3551, 0.07667],
                        [6.61366, -0.39861, 3.67395, 0.07114],
                        [6.37855, -0.27303, 4.0, 0.06664],
                        [6.12971, -0.16318, 4.0, 0.068],
                        [5.87023, -0.06607, 4.0, 0.06926],
                        [5.60332, 0.02183, 4.0, 0.07025],
                        [5.33233, 0.10457, 4.0, 0.07084],
                        [5.05886, 0.191, 4.0, 0.0717],
                        [4.78705, 0.28088, 4.0, 0.07157],
                        [4.51734, 0.37504, 4.0, 0.07142],
                        [4.25013, 0.47434, 4.0, 0.07127],
                        [3.98586, 0.57965, 4.0, 0.07112],
                        [3.72497, 0.69186, 4.0, 0.071],
                        [3.46791, 0.81186, 4.0, 0.07092],
                        [3.21519, 0.94066, 4.0, 0.07091],
                        [2.96663, 1.07788, 4.0, 0.07098],
                        [2.72207, 1.22319, 3.4734, 0.0819],
                        [2.48138, 1.3763, 2.95801, 0.09644],
                        [2.2444, 1.53695, 2.6175, 0.10938],
                        [2.01102, 1.70495, 2.36853, 0.12141],
                        [1.77826, 1.86318, 2.17493, 0.1294],
                        [1.54226, 2.00152, 2.02035, 0.1354],
                        [1.30308, 2.11012, 2.02035, 0.13002],
                        [1.06277, 2.18154, 2.02035, 0.12409],
                        [0.82507, 2.21054, 2.02035, 0.11852],
                        [0.59514, 2.1933, 2.02035, 0.11413],
                        [0.37951, 2.12639, 2.02035, 0.11175],
                        [0.18738, 2.00567, 2.25009, 0.10084],
                        [0.02041, 1.84246, 2.45642, 0.09505],
                        [-0.11979, 1.64245, 2.72557, 0.08962],
                        [-0.2327, 1.41057, 2.89357, 0.08913],
                        [-0.31952, 1.15203, 2.75521, 0.09899],
                        [-0.38268, 0.88076, 2.75521, 0.10109],
                        [-0.47068, 0.6297, 2.75521, 0.09655],
                        [-0.58325, 0.39954, 2.75521, 0.093],
                        [-0.72038, 0.19129, 2.75521, 0.0905],
                        [-0.88251, 0.0067, 2.75521, 0.08917],
                        [-1.07069, -0.15135, 2.91272, 0.08437],
                        [-1.28143, -0.28452, 2.90098, 0.08593],
                        [-1.5149, -0.3905, 2.88791, 0.08878],
                        [-1.77163, -0.46538, 2.87473, 0.09303],
                        [-2.05101, -0.50308, 2.85531, 0.09873],
                        [-2.34343, -0.49631, 2.83623, 0.10313],
                        [-2.62585, -0.44401, 2.81416, 0.10206],
                        [-2.88534, -0.35235, 2.77942, 0.09901],
                        [-3.11794, -0.22804, 2.77942, 0.09489],
                        [-3.32273, -0.0763, 2.77942, 0.0917],
                        [-3.49926, 0.09914, 2.77942, 0.08955],
                        [-3.64693, 0.2956, 2.77942, 0.08842],
                        [-3.76451, 0.51113, 2.77942, 0.08833],
                        [-3.84959, 0.74438, 2.97249, 0.08353],
                        [-3.90413, 0.99187, 3.13224, 0.08091],
                        [-3.92867, 1.2514, 2.87929, 0.09054],
                        [-3.92367, 1.52107, 2.56729, 0.10506],
                        [-3.88972, 1.79927, 2.33643, 0.11995],
                        [-3.82764, 2.0847, 2.15094, 0.1358],
                        [-3.78913, 2.36961, 2.0, 0.14375],
                        [-3.78456, 2.64214, 2.0, 0.13628],
                        [-3.81839, 2.89599, 2.0, 0.12805],
                        [-3.89168, 3.12555, 2.0, 0.12049],
                        [-4.00384, 3.32564, 2.0, 0.11469],
                        [-4.15405, 3.49017, 2.0, 0.11139],
                        [-4.34143, 3.61022, 2.18558, 0.10182],
                        [-4.55583, 3.68817, 2.32149, 0.09827],
                        [-4.79157, 3.72284, 2.47975, 0.09609],
                        [-5.0433, 3.71259, 2.67553, 0.09416],
                        [-5.30489, 3.65668, 2.92198, 0.09155],
                        [-5.56921, 3.55677, 3.24698, 0.08703],
                        [-5.82891, 3.41846, 3.70426, 0.07943],
                        [-6.07838, 3.25115, 3.86557, 0.07771],
                        [-6.3095, 3.06252, 3.94107, 0.0757],
                        [-6.51291, 2.86442, 4.0, 0.07099],
                        [-6.69062, 2.66102, 4.0, 0.06752],
                        [-6.84677, 2.45288, 4.0, 0.06505],
                        [-6.98449, 2.23986, 4.0, 0.06342],
                        [-7.10596, 2.02161, 4.0, 0.06244],
                        [-7.21267, 1.79772, 4.0, 0.062],
                        [-7.30561, 1.56769, 4.0, 0.06202],
                        [-7.38539, 1.33091, 4.0, 0.06246],
                        [-7.45231, 1.08661, 4.0, 0.06332],
                        [-7.50634, 0.83379, 4.0, 0.06463],
                        [-7.54669, 0.57065, 4.0, 0.06655],
                        [-7.57175, 0.29448, 4.0, 0.06933],
                        [-7.57891, 0.00465, 4.0, 0.07248],
                        [-7.56571, -0.28965, 4.0, 0.07365],
                        [-7.53145, -0.58194, 4.0, 0.07357],
                        [-7.47614, -0.86926, 4.0, 0.07315],
                        [-7.40002, -1.14952, 4.0, 0.0726],
                        [-7.30358, -1.42094, 4.0, 0.07201],
                        [-7.18744, -1.68192, 4.0, 0.07141],
                        [-7.05235, -1.93099, 4.0, 0.07084],
                        [-6.89934, -2.16702, 4.0, 0.07032],
                        [-6.72945, -2.38903, 4.0, 0.06989],
                        [-6.54376, -2.59622, 4.0, 0.06956],
                        [-6.34337, -2.78803, 4.0, 0.06935],
                        [-6.12949, -2.96413, 4.0, 0.06926],
                        [-5.90321, -3.12432, 4.0, 0.06931],
                        [-5.66565, -3.26856, 4.0, 0.06948],
                        [-5.41786, -3.39698, 4.0, 0.06977],
                        [-5.16091, -3.50988, 4.0, 0.07017],
                        [-4.8958, -3.60771, 4.0, 0.07065],
                        [-4.62355, -3.69116, 4.0, 0.07119],
                        [-4.34507, -3.76093, 4.0, 0.07177],
                        [-4.06124, -3.81795, 4.0, 0.07237],
                        [-3.77291, -3.86323, 4.0, 0.07297],
                        [-3.48083, -3.89793, 4.0, 0.07353],
                        [-3.18572, -3.9233, 4.0, 0.07405],
                        [-2.88823, -3.94069, 4.0, 0.0745],
                        [-2.58894, -3.95153, 4.0, 0.07487],
                        [-2.2884, -3.95732, 4.0, 0.07515],
                        [-1.98705, -3.95963, 4.0, 0.07534],
                        [-1.68528, -3.96, 4.0, 0.07544],
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
        if self.verbose:
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

        speed_diff, speed_reward = get_speed_reward()
        reward += speed_reward

        projected_time, steps_reward = get_step_reward()
        reward += steps_reward

        direction_diff, direction_reward = get_direction_reward()
        reward += direction_reward

        finish_reward = get_finish_reward()
        reward += finish_reward

        ## Zero reward if off track ##
        if is_offtrack:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            printStr = ("REWARD: {:3.4f}, DIS_REW: {:3.4f}, SPD_REW: {:3.4f}, DIR_REW: {:3.4f}, "
                        "FIN_REW: {:3.4f}, ACT_SPD: {:3.4f}, EXP_SPD: {:3.4f}, SPD_DIFF: {:3.4f}, "
                        "CLOSET_INDEX: {}, DIST: {:3.4f}, DIR_DIFF: {:3.4f}, PROJ_TIME: {:3.4f}").format(reward,
                                                                                                         distance_reward, speed_reward,
                                                                                                         direction_reward, finish_reward, speed, optimals[
                                                                                                             2], speed_diff,
                                                                                                         closest_index, dist, direction_diff, projected_time)
            print(printStr)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


# add parameter verbose=True to get noisy output for testing
reward_object = Reward(verbose=True, debug=False)


def reward_function(params):
    return reward_object.reward_function(params)
