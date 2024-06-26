class MyCar:
    def __init__(self, debug=False):
        self.pre_progress = 0
        self.pre_progress2 = 0
        self.debug = debug

    def reward_function(self, params):

        # Read input parameters
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        track_width = params['track_width']
        is_left_of_center = params['is_left_of_center']
        distance_from_center = params['distance_from_center']
        steering_angle = params['steering_angle']
        is_offtrack = params['is_offtrack']
        is_crashed = params['is_crashed']
        closest_waypoints = params['closest_waypoints']

        normDistance = distance_from_center / track_width

        reward = 0

        progress_reward = self.get_progress_reward(
            steps, speed, progress, is_offtrack, is_crashed)

        reward += progress_reward

        straight_line_reward = self.encourage_go_straight_on_straight_lines_and_speed(
            steering_angle, closest_waypoints, speed)

        reward *= straight_line_reward

        # punishment on offtrack
        if is_offtrack or is_crashed:
            reward = 1e-4

        # Avoid negative reward
        if (reward <= 0):
            reward = 1e-3

        if self.debug:
            # Print the parameter out to console (so we can see it in local training)
            carString = ('REWARD: {:3.4f}, STR_REW: {:5.4f}, PRO_REW: {:5.4f}, PRO: {:5.2f}, PRE_PRO: {:5.2f}, '
                         'PRE_PRO2: {:5.2f}, STEER:{:5.1f}, SPEED: {:3.1f}, '
                         'IS_LEFT: {}, NORMDIST: {:.2f}, DISTANCE: {:.2f}, '
                         ' TRACK_WIDTH: {:.2f}'.format(reward, straight_line_reward, progress_reward, progress,
                                                       self.pre_progress, self.pre_progress2, steering_angle,
                                                       speed, is_left_of_center, normDistance,
                                                       distance_from_center, track_width))
            print(carString)

        self.pre_progress2 = self.pre_progress
        self.pre_progress = progress

        return float(reward)

    def encourage_go_straight_on_straight_lines_and_speed(self, steering_angle, closest_waypoints, speed):
        # configure the waypoints of straignt line, this has to be configured for each track
        straight_line_waypoints = list(range(0, 22, 1)) + list(range(142, 156, 1))
        # print(straight_line_waypoints)
        closest_waypoint = closest_waypoints[1]
        STANDARD_REWARD = 1
        BAD_REWARD = 1e-3
        if closest_waypoint in straight_line_waypoints:
            MIN_SPEED = 2.0
            ABS_STEERING_THRESHOLD = 5
            if abs(steering_angle) > ABS_STEERING_THRESHOLD:
                return BAD_REWARD
            else:
                return STANDARD_REWARD * (speed / MIN_SPEED)
        else:
            return STANDARD_REWARD

    def get_progress_reward(self, steps, speed, progress, is_offtrack, is_crashed):
        # Set the minimum step that we will reward the car for high speed
        # As the begining, the higher the speed (throttle) the better, and
        # there's almost no problem when car run max speed as start, so we
        # just reward car if they start with higher speed
        MINIMUM_SPEEDING_STEPS = 4
        MINIMUM_SPEED = 3
        if (steps <= MINIMUM_SPEEDING_STEPS) and (speed >= MINIMUM_SPEED):
            reward = 2
        else:
            reward = 1e-4

        A = 4
        B = 2

        # We use two previous progress parameter, so the minimum step the car
        # must run before we can calculate the reward is 3
        # This will give reward depend on the car's progress increase speed,
        # so if car increase progress faster, it will get higher reward.
        # In other words, the faster you reach the finishing line, the higher
        # reward you'll get
        if (steps > MINIMUM_SPEEDING_STEPS):
            reward = ((progress - self.pre_progress)*A)**B + \
                     ((progress - self.pre_progress2)*0.5*A)**B

        # Sometime in local training, car teleport to random location, this
        # will lead to a huge progress reward (from 10% progress jumped to 20%
        # progress) and therefore disrupts our model. Therefore we need this
        # to remove the teleport problem
        if ((progress - self.pre_progress) >= 3) or ((progress - self.pre_progress2) >= 3) or is_offtrack or is_crashed:
            reward = 1e-4
        return reward


myCarObject = MyCar(True)


def reward_function(params):
    return myCarObject.reward_function(params)
