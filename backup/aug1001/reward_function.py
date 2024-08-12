import math
import logging
import numpy as np

# change this according to tracks
FUTURE_STEPS = 8

# Description
""" The function is used to create a line between two points in x-y coordinates,
    the points of line will be separated by the difference of distance parameter."""

# Parameters:
# x1: x coordinate of starting point
# y1: y coordinate of starting point
# x2: x coordinate of ending point
# y2: y coordinate of ending point
# distance: distance between points of lines
def get_line_points(x1, y1, x2, y2, distance=0.1):
    dx = x2 - x1
    dy = y2 - y1
    line_length = math.sqrt(dx ** 2 + dy ** 2)
    num_points = int(line_length / distance) + 1
    x_steps = dx / (num_points - 1)
    y_steps = dy / (num_points - 1)
    line_points = [(x1 + i * x_steps, y1 + i * y_steps)
                   for i in range(num_points)]
    return line_points


# Description:
""" This function takes the current waypoint and find the next x waypoints."""
def find_next_n_waypoints(params, n):
    waypoints = params['waypoints']
    next_points = (
        list(range(params['closest_waypoints'][1], params['closest_waypoints'][1] + n)))
    for i in range(len(next_points)):
        if next_points[i] >= len(waypoints):
            next_points[i] -= len(waypoints)
    return next_points

def angle_between_points(first_point, x, third_point):
    """Calculates the angle between two line segments formed by three points."""
    first_dx = first_point[0] - x
    first_dy = first_point[1] - 0
    third_dx = third_point[0] - x
    third_dy = third_point[1] - 0
    angle = math.atan2(third_dy, third_dx) - math.atan2(first_dy, first_dx)
    return math.degrees(angle)

def remove_keys(d, keys):
    return {k: v for k, v in d.items() if k not in keys}


def get_progress_score(progress, steps, speed):
    return ((progress / steps) * 100) + (speed**2)

class Reward:
    def __init__(self, verbose=False, debug=False, output_log=False):
        self.prev_steering_angle = 0
        self.first_racingpoint_index = 0
        self.prev_speed = 0
        self.debug = debug
        self.verbose = verbose
        self.intermediate_progress = [0] * 11
        self.best_reward = 0
        if output_log:
            # output to a log file
            logging.basicConfig(filename='app.log',
                                filemode='w',
                                format='%(message)s',
                                level=logging.INFO)
            self.logger = logging.getLogger('reward_logger')
        else:
            # only output to console
            logging.basicConfig(format='%(message)s',
                                level=logging.INFO)
            self.logger = logging.getLogger('reward_logger')

    def identify_corner(self, params):
        future_step = 5
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']

        # Identify next waypoint and a further waypoint
        point_prev = waypoints[closest_waypoints[0]]
        point_next = waypoints[closest_waypoints[1]]
        point_future = waypoints[min(len(waypoints) - 1, closest_waypoints[1] + future_step)]

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

        return diff_heading
    
    def get_direction_reward(self, params):
        diff_heading = self.identify_corner(params)
        TURN_THRESHOLD_STRAIGHT = 25    # degrees
        is_straight = True
        if diff_heading > TURN_THRESHOLD_STRAIGHT:
            is_straight = False

        if not is_straight:
            return self.get_direction_reward2(params)
        else:
            return self.get_direction_reward_on_straight(params)

    def get_direction_reward2(self, params):
        waypoints = params['waypoints']
        heading = params['heading']
        x = params['x']
        y = params['y']

        next_points = self.get_optimal_line(params)
        x_forward = waypoints[next_points[FUTURE_STEPS-1]][0]
        y_forward = waypoints[next_points[FUTURE_STEPS-1]][1]
        # Calculate reward for alignment with optimal steering direction
        optimal_heading = math.degrees(math.atan2(y_forward - y, x_forward - x))
        heading_diff = abs(optimal_heading - heading)
        
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        
        error = (heading_diff) / 60.0  # 60 degree is already really bad
        score = 1.0 - abs(error)
        final_reward = max(score, 1e-3)
        if self.debug and self.verbose:
            self.logger.info('forward_point: %s, optimal_heading: %s, heading %s, final_reward %s', next_points[3], optimal_heading, heading, final_reward)
        return final_reward

    def get_direction_reward_on_straight(self, params):
        next_points = self.get_optimal_line(params)
        heading = params['heading']
        waypoints = params['waypoints']
        x1_forward = waypoints[next_points[0]][0]
        y1_forward = waypoints[next_points[0]][1]
        x3_forward = waypoints[next_points[FUTURE_STEPS-1]][0]
        y3_forward = waypoints[next_points[FUTURE_STEPS-1]][1]
        optimal_heading = math.degrees(math.atan2(y3_forward - y1_forward, x3_forward - x1_forward))
        heading_diff = abs(optimal_heading - heading)
        
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        
        error = (heading_diff) / 60.0  # 60 degree is already really bad
        score = 1.0 - abs(error)
        final_reward = max(score, 1e-3)

        if self.debug and self.verbose:
            self.logger.info('on straight lane, optimal_heading: %s, heading %s, final_reward %s', optimal_heading, heading, final_reward)
        return final_reward
    
    def get_speed_reward(self, params):
        next_points, x_forward, y_forward = self.get_optimal_line(params)
        x = params['x']
        waypoints = params['waypoints']
        speed = params['speed']
        first_point = waypoints[next_points[0]]
        third_point = waypoints[next_points[2]]
        curvature = abs(angle_between_points(first_point, x, third_point))

        # Optimal speed based on curvature
        min_speed, max_speed = 1, 4
        # Changed to continuous function for optimal speed calculation
        optimal_speed = max_speed - (curvature / 60) * (max_speed - min_speed)

        # Calculate reward for speed
        speed_diff = abs(speed - optimal_speed)
        reward_speed = math.exp(-0.5 * speed_diff)
        if self.debug and self.verbose:
            self.logger.info('curvature %s, optimal_speed %s, speed %s', curvature, optimal_speed, speed)
        return reward_speed

    def get_progress_reward(self, params):
        steps = params['steps']
        progress = params['progress']
        speed = params['speed']
        fastest_complete_time = 13 # get from the lead board
        racing_line_length = 42.25 # customized to the racing line
        fastest_average_speed = racing_line_length / fastest_complete_time
        fastest_completed_steps = 15 * fastest_complete_time
        max_reward = get_progress_score(100, fastest_completed_steps, fastest_average_speed)
        my_reward = get_progress_score(progress, steps, speed)
        if self.debug and self.verbose:
            self.logger.info('my reward {:3.5f}, max_reward {:3.5f}, progress {:3.5f}, steps {:3.5f}, speed {:3.5f}'.format(my_reward, max_reward, progress, steps, speed))
        reward = my_reward / max_reward # rescale to 1, 1 is the max possible reward
        return reward
        
    def get_progress_reward2(self, params):
        progress = params['progress']
        steps = params['steps']
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

    def get_finish_reward(self, params):
        progress = params['progress']
        return progress / 100        

    def get_optimal_line(self, params):
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

    def get_starting_straight_reward(self, params):
        speed = params['speed']
        close_waypoints = params['closest_waypoints']
        steering_angle = params['steering_angle']
        prev_speed = self.prev_speed
        self.prev_speed = speed
        HIGH_SPEED_THRESHOLD = 3
        if 0 <= close_waypoints[1] <= 20: # at begining of the starting points, encourage straight line and speed
            if steering_angle == 0 and speed >= HIGH_SPEED_THRESHOLD:
                return 1
            else:
                return 1e-3
        return 1
    
    def smooth_reward(self, params):
        prev_steering_angle = self.prev_steering_angle
        steering_angle = params['steering_angle']
        self.prev_steering_angle = steering_angle
        steering_diff = abs(steering_angle - prev_steering_angle)
        reward_steering_smoothness = math.exp(-0.05 * steering_diff)
        if self.debug and self.verbose:
            self.logger.info('steering_angle %s, prev_steering_angle %s', steering_angle, prev_steering_angle)
        return reward_steering_smoothness

    def reward_function(self, params):
        ################## INPUT PARAMETERS ###################
        is_offtrack = params['is_offtrack']
        # x = params['x']
        # y = params['y']
        # heading = params['heading']
        # progress = params['progress']
        # steps = params['steps']
        # speed = params['speed']
        # track_width = params['track_width']
        # steering_angle = params['steering_angle']
        # waypoints = params['waypoints']
        # closest_waypoints = params['closest_waypoints']
        # all_wheels_on_track = params["all_wheels_on_track"]
        # distance_from_center = params['distance_from_center']
        # is_left_of_center = params['is_left_of_center']

        if self.debug:
            print_params = remove_keys(params, ['waypoints', 'closest_objects', 'objects_location', 'objects_left_of_center',
                                                'object_in_camera', 'objects_speed', 'objects_heading', 'objects_distance_from_center', 'objects_distance',
                                                'projection_distance'])
            self.logger.info("params: %s", print_params)

        reward = 1  # basic reward to make sure it is better than offtrack
        direction_weight = 1
        speed_weight = 0
        progress_weight = 1
        smooth_weight = 0
        starting_straight_weight = 0
        finish_weight = 0

        if direction_weight > 0:
            direction_reward = self.get_direction_reward2(params)
        else:
            direction_reward = 0
        
        if speed_weight > 0:
            speed_reward = self.get_speed_reward(params)
        else:
            speed_reward = 0

        if progress_weight > 0:
            progress_reward = self.get_progress_reward2(params)
        else:
            progress_reward = 0

        if smooth_weight > 0:
            smooth_reward = self.smooth_reward(params)
        else:
            smooth_reward = 0

        if starting_straight_weight > 0:
            starting_straight_reward = self.get_starting_straight_reward(params)
        else:
            starting_straight_reward = 0

        if finish_weight > 0:
            finish_reward = self.get_finish_reward(params)
        else:
            finish_reward = 0        

        if is_offtrack:
            reward = 1e-3
        else:
            reward += direction_reward * direction_weight \
                + speed_reward * speed_weight \
                + progress_reward * progress_weight \
                + smooth_reward * smooth_weight \
                + starting_straight_reward * starting_straight_weight \
                + finish_reward * finish_weight
        
        if reward > self.best_reward:
            self.best_reward = reward
        output_str = ''
        output_str += '{:3.5f}'.format(reward)
        if direction_weight > 0:
            output_str += '|{:3.5f}'.format(direction_reward)
        
        if speed_weight > 0:
            output_str += '|{:3.5f}'.format(speed_reward)
        
        if progress_weight > 0:
            output_str += '|{:3.5f}'.format(progress_reward)
        
        if smooth_weight > 0:
            output_str += '|{:3.5f}'.format(smooth_reward)
        
        if starting_straight_weight > 0:
            output_str += '|{:3.5f}'.format(starting_straight_reward)
        
        if finish_weight > 0:
            output_str += '|{:3.5f}'.format(finish_reward)

        output_str += '|best_reward:{:3.5f}'.format(self.best_reward)
            
        if self.debug:
            self.logger.info(output_str)
        return reward


# debug = True to print any nessasary logs
# verbose = True to print detailed logs for each steps
# output_log = True to save logs to file, output_log = False to print to console
reward_object = Reward(debug=True, verbose=False, output_log=True)

def reward_function(params):
    return reward_object.reward_function(params)
