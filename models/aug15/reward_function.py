# Fix the bug of version 3, adding more loggings
import math
import numpy as np
import logging


class Reward:
    def __init__(self, verbose=False, debug=False, output_log=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose
        self.debug = debug
        self.intermediate_progress = [0]*11
        self.best_reward = 0 # best reward per episode
        self.max_curvature = 0
        self.all_optimal_speed_set = set()
        if output_log:
            # Configure the logger
            logging.basicConfig(filename='app.log',
                                filemode='w',
                                format='%(message)s',
                                level=logging.INFO)

            # Create a logger
            self.logger = logging.getLogger('my_logger')
        else:
            # Configure the logger
            logging.basicConfig(format='%(message)s',
                                level=logging.INFO)

            # Create a logger
            self.logger = logging.getLogger('my_logger')

    def reward_function(self, params):

        FUTURE_STEPS = 8
        TARGET_WAYPOINT = 5
        SPEED_THRESHOLD_SLOW = 1.5  # m/s
        SPEED_THRESHOLD_FAST = 2.8    # m/s
        STEERING_THRESHOLD = 5     # degrees
        TURN_THRESHOLD_STRAIGHT = 6    # degrees
        FUTURE_STEP_STRAIGHT = 6

        #################################################################
        ###                 Helper Functions                          ###
        #################################################################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_racing_points_index(racing_coords, car_coords):

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
            second_closest_index = distances_no_closest.index(min(distances_no_closest))
            closest_points_index = [closest_index, second_closest_index]
            if self.debug and self.verbose:
                self.logger.info('closet_points_index: %s', closest_points_index)
            return closest_points_index

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
        

        def get_track_direction_in_degree(p1, p2):
            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(p2[1] - p1[1], p2[0] - p1[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)
            return track_direction
        

        def get_track_direction(closest_coords, second_closest_coords, car_coords):
            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            return get_track_direction_in_degree(prev_point, next_point)

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            track_direction = get_track_direction(closest_coords, second_closest_coords, car_coords)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            if self.debug and self.verbose:
                self.logger.info('track_direction %s, heading %s, direction_diff %s', track_direction, heading, direction_diff)
            return direction_diff, track_direction
        
        def racing_direction_diff2(p1_coords, p2_coords, heading):

            track_direction = get_track_direction_in_degree(p1_coords, p2_coords)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            if self.debug and self.verbose:
                self.logger.info('track_direction2 %s, heading %s, direction_diff %s', track_direction, heading, direction_diff)
            return direction_diff, track_direction

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

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical( first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum( [times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time /
                                  current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time
        
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
        
        def is_in_corner(waypoints, closest_waypoints):
            diff_heading, dist_future = identify_corner(waypoints, closest_waypoints, 6)
            if diff_heading < TURN_THRESHOLD_STRAIGHT:
                return False
            else:
                return True

        def select_speed(waypoints, closest_waypoints, future_step):

            # Identify if a corner is in the future
            diff_heading, dist_future = identify_corner(waypoints,
                                                        closest_waypoints,
                                                        future_step)

            if diff_heading < TURN_THRESHOLD_STRAIGHT:
                # If there's no corner encourage going faster
                go_fast = True
            else:
                # If there is a corner encourage slowing down
                go_fast = False

            return go_fast
        
        def select_straight():

            # Identify if a corner is in the future
            diff_heading, dist_future = identify_corner(waypoints,
                                                        closest_waypoints,
                                                        FUTURE_STEP_STRAIGHT)

            if diff_heading < TURN_THRESHOLD_STRAIGHT:
                # If there's no corner encourage going straighter
                go_straight = True
            else:
                # If there is a corner don't encourage going straighter
                go_straight = False

            return go_straight
        
        def select_straight2():
            if ((90 <= closest_waypoints[1] <= 102)
                    or (112 <= closest_waypoints[1] <= 130)
                    or (139 <= closest_waypoints[1] <= 141)
                    or (0 <= closest_waypoints[1] <= 22)
                    or (41 <= closest_waypoints[1] <= 47)
            ):
                return True
            else:
                return False

        """ This function takes the current waypoint and find the next x waypoints."""
        def find_next_n_waypoints(params, n):
            waypoints = params['waypoints']
            next_points = (
                list(range(params['closest_waypoints'][1], params['closest_waypoints'][1] + n)))
            for i in range(len(next_points)):
                if next_points[i] >= len(waypoints):
                    next_points[i] -= len(waypoints)
            return next_points

        def get_next_points(params):
            # Get current position
            # x = params['x']
            # y = params['y']
            # waypoints = params['waypoints']

            next_points = find_next_n_waypoints(params, FUTURE_STEPS)
            if self.debug and self.verbose:
                self.logger.info('next_points: %s', next_points)
                pass

            # Get Destination coordinates
            # x_forward = waypoints[next_points[2]][0]
            # y_forward = waypoints[next_points[2]][1]

            # optimal_path = get_line_points(x, y, x_forward, y_forward)
            return next_points
        
        def angle_between_points(first_point, second_point, third_point):
            """Calculates the angle between two line segments formed by three points."""
            ab_x = second_point[0] - first_point[0]
            ab_y = second_point[1] - first_point[1]
            bc_x = third_point[0] - second_point[0]
            bc_y = third_point[1] - second_point[1]
            angle = math.atan2(bc_y, bc_x) - math.atan2(ab_y, ab_x)
            angle_in_degree = math.degrees(angle)
            if self.debug and self.verbose:
                # self.logger.info('angle %s, angle_in_degree %s', angle, angle_in_degree)
                pass
            return angle_in_degree
        

        def get_racing_line_curvrature(first_point, second_point, third_point):
            return angle_between_points(first_point, second_point, third_point)
        
        # by looking at the closet racing line points, find the array of [x, x+1 and x+n], then use the 3 points to calculate the curvature
        def get_closest_index(index1, index2):
            index3 = max(index1, index2) + LOOK_FORWARD_STEPS
            if index3 >= len(racing_track):
                index3 = index3 - len(racing_track)
            
            self.logger.info('## closest_index %s, second_closest_index %s, third index %s', index1, index2, index3)
            if index2 > index1:
                return [index1, index2, index3]
            else:
                return [index2, index1, index3]

        #################################################################
        ###      Below are the functions to calculate the rewards     ###
        #################################################################
        def get_distance_reward():
            ## Reward if car goes close to optimal racing line ##
            dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
            distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
            if self.debug and self.verbose:
                self.logger.info('distance_reward %s, dist %s', distance_reward, dist)
            return dist, distance_reward

        #################################################################
        # Using the racing line to calculate the direction
        def get_direction_reward():
            if direction_diff > STEERING_ANGLE_THRESHOLD:
                reward = 1e-3
            else:
                reward = 1.0 - direction_diff / STEERING_ANGLE_THRESHOLD
            if self.debug and self.verbose:
                self.logger.info('direction_reward %s, direction_diff %s', reward, direction_diff)
            return direction_diff, reward
        
        # Using the racing line to calculate the direction
        # and also consider the good move of the steering angle
        def get_direction_reward2():
            punish_weight = 1
            good_steer_move = True
            ahead_track_diff = ahead_track_direction - heading
            if abs(direction_diff_ahead) > 10: # in the corner
                if ahead_track_diff < 0 and steering_angle >= 5: # turn left but steer right then not good move
                    punish_weight = 2
                    good_steer_move = False
                elif ahead_track_diff > 0 and steering_angle <= -5: # turn right but steer left then not good move
                    punish_weight = 2
                    good_steer_move = False

            if direction_diff > STEERING_ANGLE_THRESHOLD:
                reward = 1e-3
            else:
                reward = max(1.0 - direction_diff / STEERING_ANGLE_THRESHOLD * punish_weight, 1e-3)
            if self.debug and self.verbose:
                self.logger.info('direction_reward %s, direction_diff %s, ahead_track_diff %s, steering_angle %s, good steer move? %s, punish_weight %s', 
                                 reward, direction_diff, ahead_track_diff, steering_angle, good_steer_move, punish_weight)
            return direction_diff, reward

        # using the waypoints to calculate the direction, this may not be accurate since we don't follow the centerline 
        # waypoints, so discard of the implementation
        def get_direction_reward3():
            waypoints = params['waypoints']
            heading = params['heading']
            x = params['x']
            y = params['y']

            next_points = get_next_points(params)
            target_forward_wp = TARGET_WAYPOINT-1
            x_forward = waypoints[next_points[target_forward_wp]][0]
            y_forward = waypoints[next_points[target_forward_wp]][1]
            # Calculate reward for alignment with optimal steering direction
            optimal_heading = math.degrees(math.atan2(y_forward - y, x_forward - x))
            heading_diff = abs(optimal_heading - heading)
            
            if heading_diff > 180:
                heading_diff = 360 - heading_diff
            
            error = (heading_diff) / 60.0  # 60 degree is already really bad
            score = 1.0 - abs(error)
            final_reward = max(score, 1e-3)
            if self.debug and self.verbose:
                self.logger.info('forward_point: %s, optimal_heading: %s, heading %s, final_reward %s', next_points[target_forward_wp], optimal_heading, heading, final_reward)
            return 0, final_reward
        
        #################################################################
        # use the optimized the speed
        def get_speed_reward():
            SPEED_DIFF_NO_REWARD = 1
            speed_diff = abs(optimals[2]-speed)
            if speed_diff <= SPEED_DIFF_NO_REWARD:
                # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
                # so, we do not punish small deviations from optimal speed
                speed_reward = math.exp(-1.5 * speed_diff)
                # speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
            else:
                speed_reward = 1e-3
            if self.debug and self.verbose:
                self.logger.info('speed %s, speed_diff %s, speed_reward %s', speed, speed_diff, speed_reward)
            return speed_diff, speed_reward

        # according to the current race line and ahead race line angle, calculate the speed reward
        # the more curve, the lower speed   
        def get_speed_reward2():
            ahead_track_diff = ahead_track_direction - heading
            abs_ahead_track_diff = abs(ahead_track_diff)
            if abs_ahead_track_diff > 90:
                ahead_track_diff = 180 - abs_ahead_track_diff

            # Optimal speed based on curvature
            min_speed, max_speed = 1.5, 4
            punish_score = abs_ahead_track_diff / MAX_CURVATURE
            if abs_ahead_track_diff >= 50: # 60 degrees is already a big curve
                punish_score = 1 # if it is already too much, then will reduced to the lowest speed as possible
            # Changed to continuous function for optimal speed calculation
            optimal_speed = round(max_speed - punish_score * (max_speed - min_speed), 1)
            self.all_optimal_speed_set.add(optimal_speed)

            # Calculate reward for speed
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_diff = abs(speed - optimal_speed)
            reward_speed = math.exp(-1.5 * speed_diff)
            if abs_ahead_track_diff > self.max_curvature:
                self.max_curvature = abs_ahead_track_diff
            if self.debug and self.verbose:
                self.logger.info('speed_reward %s, curvature %s, optimal_speed %s, speed %s', reward_speed, abs_ahead_track_diff, optimal_speed, speed)
            return speed_diff, reward_speed

        # Use the track shape to decide the curvature of the track, this may not be accurate since we don't follow the center line
        def get_speed_reward3():
            ## according to the track shape
            speed = params['speed']
            waypoints = params['waypoints']
            next_points = get_next_points(params)
            first_point_wp = next_points[0]
            second_point_wp = next_points[1]
            third_point_wp = next_points[6]
            first_point = waypoints[first_point_wp]
            second_point = waypoints[second_point]
            thrid_point = waypoints[thrid_point]
            curvature = get_racing_line_curvrature(first_point, second_point, thrid_point)
            abs_curvature = abs(curvature)

            # Optimal speed based on curvature
            min_speed, max_speed = 1.5, 4
            MAX_CURVATURE = 50 # based on the shape of the track
            # Changed to continuous function for optimal speed calculation
            optimal_speed = round(max_speed - (abs_curvature / MAX_CURVATURE) * (max_speed - min_speed), 1)
            self.all_optimal_speed_set.add(optimal_speed)

            # Calculate reward for speed
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_diff = abs(speed - optimal_speed)
            reward_speed = math.exp(-1.5 * speed_diff)
            if self.debug and self.verbose:
                self.logger.info('curvature %s, (%s, %s, %s), optimal_speed %s, speed %s', curvature, first_point_wp, second_point_wp, third_point_wp, optimal_speed, speed)
            max_curvature, _, _, _ = self.max_curvature
            if abs_curvature > max_curvature:
                self.max_curvature = (abs_curvature, first_point_wp, second_point_wp, third_point_wp)
            return speed_diff, reward_speed

        # Just based on the corner to decide go with high speed or low speed
        # This may not be precise enough       
        def get_speed_reward4():
            stay_straight = select_straight2()
            if stay_straight and speed >= SPEED_THRESHOLD_FAST and abs(steering_angle) <= STEERING_THRESHOLD:
                reward = 1
            elif not stay_straight and speed <= SPEED_THRESHOLD_SLOW:
                reward = 1
            else:
                reward = 1e-3
            if self.debug and self.verbose:
                self.logger.info('stay_straight %s, speed %s', stay_straight, speed)
            return 0, reward

        #################################################################
        def get_step_reward():
            # Reward if less steps
            REWARD_PER_STEP_FOR_FASTEST_TIME = 1
            FASTEST_TIME = 13  # seconds (best time of 1st place on the track)
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

        #################################################################
        def get_finish_reward():
            ## Incentive for finishing the lap in less steps ##
            STANDARD_TIME = 25  # seconds (time that is easily done by model)
            if progress == 100:
                finish_reward = max(1e-3, 1 * (STANDARD_TIME*15-steps))
            else:
                finish_reward = 1e-3
            return finish_reward
        
        #################################################################
        def get_straight_reward():
            stay_straight = select_straight()
            if stay_straight and abs(steering_angle) < STEERING_THRESHOLD:
                return 1
            elif abs(steering_angle) > STEERING_THRESHOLD: # discourage for zigzagging on straight line
                return 1e-3
            else:
                return 0.5

        #################################################################
        def get_progress_reward():
            fastest_complete_time = 12.4 # get from the lead board
            racing_line_length = 42.25 # customized to the racing line
            fastest_average_speed = racing_line_length / fastest_complete_time
            fastest_completed_steps = 15 * fastest_complete_time
            max_reward = ((100 / fastest_completed_steps) * 100) + (fastest_average_speed**2)
            if all_wheels_on_track and steps > 0:
                my_reward = ((progress / steps) * 100) + (speed**2)
                self.logger.info('my reward {:3.5f}, max_reward {:3.5f}, progress {:3.5f}, steps {:3.5f}, speed {:3.5f}'.format(my_reward, max_reward, progress, steps, speed))
                reward = my_reward / max_reward # rescale to 1, 1 is the max possible reward
            else:
                reward = 1e-3
            return reward
        
        def get_progress_reward2():
            # Reward for making steady progress
            # Bonus that the agent gets for completing every 10 percent of track
            # Is exponential in the progress / steps. 
            # exponent increases with an increase in fraction of lap completed
            intermediate_progress_bonus = 0
            best_bonus_score = 1
            best_time = 12.4 # get it from lead board
            best_steps = best_time * 15 + 1
            best_progress_steps = 100 / best_steps
            my_progress_steps = progress / steps
            my_progress_score = my_progress_steps / best_progress_steps * best_bonus_score
            pi = int(progress/10)
            MAX_PROGRESS_SCORE = 100**2
            if pi==10: # 100% track completion
                if self.debug:
                    self.logger.info('completion the lapse!')
                completion_bonus = 2 #bigger incentive bonus for completing the track
                self.intermediate_progress[pi] = completion_bonus
                intermediate_progress_bonus = completion_bonus
            elif pi != 0 and self.intermediate_progress[pi] == 0:
                intermediate_progress_bonus = progress ** 2 / MAX_PROGRESS_SCORE
                self.intermediate_progress[pi] = intermediate_progress_bonus

            if self.debug and self.verbose:
                self.logger.info('pi=%s, bonus %s, intermediate_progress %s, my_progress_score %s, progress %s, steps %s', 
                                 pi, self.intermediate_progress[pi], my_progress_score, self.intermediate_progress, progress, steps)
            return self.intermediate_progress[pi] + my_progress_score

        #################################################################
        def rescale_reward(reward, distance_reward, distance_weight, direction_reward, direction_weight, speed_reward, speed_weight, progress_reward, progress_weight):
            # reward += direction_reward * direction_weight \
            #     + distance_reward * distance_weight \
            #     + speed_reward * speed_weight \
            #     + progress_reward * progress_weight
            
            reward += (direction_reward * direction_weight \
                + distance_reward * distance_weight \
                + speed_reward * speed_weight)**2 + direction_reward * distance_reward * speed_reward
            return reward

    
    #################### RACING LINE ######################

        # Optimal racing line
        # Each row: [x,y,speed,timeFromPreviousPoint]
        # Custom configs for each track
        AHEAD_STEPS = 3 # to calculate the angle between the ahead racing line and current racing line 
        STEERING_ANGLE_THRESHOLD = 30 # to decide which is the maximum steering angle between the optimized racing line and current heading
        LOOK_FORWARD_STEPS = 6
        MAX_CURVATURE = 50 # based on the shape of the track, what could be the maximum curvature of the racing line
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
                        [1.57433, 3.36728, 3.67586, 0.07877],
                        [1.28771, 3.35904, 3.18588, 0.09],
                        [1.00472, 3.34042, 2.7954, 0.10145],
                        [0.72621, 3.30944, 2.7954, 0.10025],
                        [0.45323, 3.26372, 2.7954, 0.09901],
                        [0.18727, 3.20011, 2.7954, 0.09783],
                        [-0.06923, 3.11357, 2.7954, 0.09684],
                        [-0.31295, 2.99814, 3.4533, 0.07809],
                        [-0.54791, 2.86396, 4.0, 0.06764],
                        [-0.77701, 2.71753, 4.0, 0.06797],
                        [-1.00333, 2.56535, 4.0, 0.06818],
                        [-1.23691, 2.41275, 4.0, 0.06975],
                        [-1.47314, 2.26391, 4.0, 0.0698],
                        [-1.71256, 2.12036, 3.67251, 0.07601],
                        [-1.95558, 1.9837, 3.45776, 0.08063],
                        [-2.20267, 1.8559, 3.45776, 0.08045],
                        [-2.45443, 1.73953, 3.45776, 0.08021],
                        [-2.71151, 1.63741, 3.45776, 0.08],
                        [-2.97486, 1.5534, 3.45776, 0.07994],
                        [-3.24483, 1.49004, 3.91823, 0.07077],
                        [-3.51965, 1.44306, 4.0, 0.0697],
                        [-3.79826, 1.40978, 4.0, 0.07015],
                        [-4.07993, 1.38815, 4.0, 0.07062],
                        [-4.36403, 1.37636, 4.0, 0.07109],
                        [-4.65002, 1.37253, 4.0, 0.0715],
                        [-4.93738, 1.37477, 4.0, 0.07184],
                        [-5.22573, 1.3816, 3.57656, 0.08065],
                        [-5.50801, 1.3917, 3.10473, 0.09098],
                        [-5.78852, 1.39663, 2.67789, 0.10477],
                        [-6.06562, 1.39176, 2.35617, 0.11762],
                        [-6.3377, 1.3728, 2.35617, 0.11575],
                        [-6.6028, 1.33517, 2.35617, 0.11364],
                        [-6.85855, 1.2742, 2.35617, 0.11158],
                        [-7.10111, 1.1837, 2.35617, 0.10988],
                        [-7.32482, 1.05732, 2.7268, 0.09423],
                        [-7.53259, 0.90544, 2.6401, 0.09748],
                        [-7.72388, 0.73098, 2.44982, 0.10568],
                        [-7.8974, 0.53608, 2.24905, 0.11603],
                        [-8.04967, 0.32165, 1.99556, 0.13179],
                        [-8.17647, 0.09006, 1.76932, 0.14923],
                        [-8.27229, -0.15474, 1.56276, 0.16822],
                        [-8.33019, -0.4062, 1.56276, 0.16512],
                        [-8.34431, -0.65461, 1.56276, 0.15921],
                        [-8.30911, -0.88837, 1.56276, 0.15127],
                        [-8.22184, -1.09536, 1.56276, 0.14374],
                        [-8.08127, -1.26037, 1.67702, 0.12926],
                        [-7.90314, -1.38332, 1.88686, 0.11471],
                        [-7.69903, -1.46867, 2.09104, 0.1058],
                        [-7.47551, -1.51907, 2.3533, 0.09737],
                        [-7.23778, -1.53788, 2.6211, 0.09098],
                        [-6.98954, -1.52773, 2.95326, 0.08412],
                        [-6.73394, -1.49196, 3.04911, 0.08464],
                        [-6.47343, -1.43427, 2.78259, 0.09589],
                        [-6.21428, -1.35829, 2.47353, 0.10918],
                        [-5.95834, -1.30327, 2.1968, 0.11917],
                        [-5.70555, -1.27032, 2.1968, 0.11605],
                        [-5.45609, -1.26088, 2.1968, 0.11364],
                        [-5.21075, -1.27818, 2.1968, 0.11196],
                        [-4.97131, -1.3282, 2.1968, 0.11135],
                        [-4.74125, -1.4197, 2.60165, 0.09516],
                        [-4.51948, -1.5419, 3.24884, 0.07794],
                        [-4.30381, -1.68494, 4.0, 0.0647],
                        [-4.09119, -1.83808, 4.0, 0.06551],
                        [-3.86599, -1.99828, 4.0, 0.06909],
                        [-3.63852, -2.15631, 4.0, 0.06924],
                        [-3.40772, -2.31086, 3.64437, 0.07622],
                        [-3.17271, -2.46047, 3.64437, 0.07645],
                        [-2.93269, -2.60342, 3.64437, 0.07666],
                        [-2.68695, -2.73763, 3.64437, 0.07683],
                        [-2.4347, -2.86063, 3.64437, 0.07701],
                        [-2.17307, -2.96577, 3.99704, 0.07054],
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
                        [1.82015, -3.36188, 3.87588, 0.07457],
                        [2.10715, -3.33249, 3.45359, 0.08354],
                        [2.39227, -3.29319, 3.45359, 0.08334],
                        [2.67468, -3.24212, 3.45359, 0.0831],
                        [2.95317, -3.17667, 3.45359, 0.08283],
                        [3.22615, -3.09368, 3.45359, 0.08262],
                        [3.49134, -2.989, 3.87765, 0.07352],
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
                        [7.15708, -0.09943, 3.47835, 0.08631],
                        [7.36075, 0.12204, 2.86722, 0.10494],
                        [7.55989, 0.34831, 2.49399, 0.12086],
                        [7.75296, 0.58004, 2.22822, 0.13537],
                        [7.93175, 0.81611, 1.9412, 0.15255],
                        [8.0842, 1.05542, 1.70781, 0.16614],
                        [8.20137, 1.29559, 1.5, 0.17816],
                        [8.27842, 1.53346, 1.5, 0.16669],
                        [8.31305, 1.76572, 1.5, 0.15655],
                        [8.30027, 1.98795, 1.5, 0.1484],
                        [8.23384, 2.19368, 1.5, 0.14412],
                        [8.10225, 2.3699, 1.85827, 0.11835],
                        [7.92766, 2.5196, 2.13104, 0.10792],
                        [7.7188, 2.64276, 2.42887, 0.09983],
                        [7.48516, 2.73902, 2.85184, 0.08861],
                        [7.24296, 2.81019, 3.35523, 0.07524],
                        [7.01403, 2.85965, 3.86615, 0.06058]]

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
        all_wheels_on_track = params["all_wheels_on_track"]
        # distance_from_center = params['distance_from_center']
        # is_left_of_center = params['is_left_of_center']

        # Read all input parameters
        if self.debug and self.verbose:
            print_params = remove_keys(params, ['waypoints', 'closest_objects', 'objects_location', 'objects_left_of_center',
                                          'object_in_camera', 'objects_speed', 'objects_heading', 'objects_distance_from_center', 'objects_distance',
                                          'projection_distance'])
            self.logger.info("params: %s", print_params)

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_racing_points_index(racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # this is to get the waypoints ahead, to calculate the steering angle of current position
        ahead_closest_index = closest_index + AHEAD_STEPS
        if ahead_closest_index >= len(racing_track):
            ahead_closest_index = ahead_closest_index - len(racing_track)
        ahead_closest_index_2 = ahead_closest_index + 1
        if ahead_closest_index_2 >= len(racing_track):
            ahead_closest_index_2 = ahead_closest_index_2 - len(racing_track)
        optimals_ahead = racing_track[ahead_closest_index]
        optimals_second_ahead = racing_track[ahead_closest_index_2]
        if self.debug and self.verbose:
            self.logger.info('ahead_closest_index: %s, ahead_closest_index_2: %s, %s, %s', 
                         ahead_closest_index, ahead_closest_index_2, racing_track[ahead_closest_index], racing_track[ahead_closest_index_2])

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff, track_direction = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
        direction_diff_ahead, ahead_track_direction = racing_direction_diff2(optimals_ahead[0:2], optimals_second_ahead[0:2], heading)
        # for the use of calculation for the current racing line curvature
        race_line_point1_index, race_line_point2_index, race_line_point3_index = get_closest_index(closest_index, second_closest_index)

        race_line_point1_cords = racing_track[race_line_point1_index]
        race_lint_point2_cords = racing_track[race_line_point2_index]
        race_lint_point3_cords = racing_track[race_line_point3_index]
        race_line_curvature = get_racing_line_curvrature(race_line_point1_cords, race_lint_point2_cords, race_lint_point3_cords)
        if self.debug and self.verbose:
            self.logger.info('## racing line curvature %s, race_line_point1_index %s, race_line_point2_index %s, race_line_point3_index %s', race_line_curvature, race_line_point1_index, race_line_point2_index, race_line_point3_index)

        # Save first racingpoint of episode for later
        if steps == 2.0:
            # when steps == 1, status is prepare, thus reset the first_racingpoint_index when steps = 2
            if self.debug and self.verbose:
                self.logger.info("set first_racingpoint_index to %i", closest_index)
            self.first_racingpoint_index = closest_index
        else:
            # print('first_racingpoint_index: %i' % self.first_racingpoint_index)
            pass

        ################ REWARD AND PUNISHMENT ################

        reward = 1  # basic reward to make sure there is penalty of offtrack
        distance_enable = True
        direction_enable = True
        speed_enable = True
        progress_enable = False
        
        distance_weight = 0
        direction_weight = 0
        speed_weight = 0
        progress_weight = 0

        if not select_straight2():
            if self.debug and self.verbose:
                self.logger.info('corner %s', closest_waypoints)
            distance_weight = 1
            direction_weight = 1
            speed_weight = 1
        else:
            if self.debug and self.verbose:
                self.logger.info('straight %s', closest_waypoints)
            distance_weight = 1
            direction_weight = 1
            speed_weight = 1

        if distance_enable:
            dist, distance_reward = get_distance_reward()
        else:
            direction_reward = 0

        if direction_enable:
            direction_diff, direction_reward = get_direction_reward2()
        else:
            direction_reward = 0

        if speed_enable:
            speed_diff, speed_reward = get_speed_reward2()
        else:
            speed_reward = 0

        if progress_enable:
            progress_reward = get_progress_reward2()
        else:
            progress_reward = 0

        if is_offtrack:
            reward = 1e-3
        else:
            reward = rescale_reward(reward, distance_reward, distance_weight, direction_reward, direction_weight, speed_reward, speed_weight, progress_reward, progress_weight)
        
        if reward > self.best_reward:
            self.best_reward = reward
        
        output_str = ''
        output_str += '{:3.5f}'.format(reward)
        
        if distance_weight > 0:
            output_str += '|{:d}x{:3.5f}'.format(distance_weight, distance_reward)

        if direction_weight > 0:
            output_str += '|{:d}x{:3.5f}'.format(direction_weight, direction_reward)
        
        if speed_weight > 0:
            output_str += '|{:d}x{:3.5f}'.format(speed_weight, speed_reward)
        
        if progress_weight > 0:
            output_str += '|{:d}x{:3.5f}'.format(progress_weight, progress_reward)
        
        output_str += '|best_reward:{:3.5f}|max_curvature:{}|all_optimal_speed_set:{}\n-----------------------'.format(self.best_reward, self.max_curvature, self.all_optimal_speed_set)

        if self.debug:
            self.logger.info(output_str)

        #################### RETURN REWARD ####################
        # Always return a float value
        return float(reward)

# debug = True to print any nessasary logs
# verbose = True to print detailed logs for each steps
# output_log = True to save logs to file, output_log = False to print to console
reward_object = Reward(debug=False, verbose=False, output_log=True)


def reward_function(params):
    return reward_object.reward_function(params)
