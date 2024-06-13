# -*- coding: utf-8 -*-
"""
AWS DeepRacer reward function
link: https://github.com/Usin2705/DeepRacer/blob/master/2019/05-ProgressVelocity-08.610.py
"""

class MyCar:
    def __init__ (self, debug = False):
        self.previous_steps = None
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

        normDistance = distance_from_center/track_width     
        
        # Set the minimum step that we will reward the car for high speed
        # As the begining, the higher the speed (throttle) the better, and 
        # there's almost no problem when car run max speed as start, so we
        # just reward car if they start with higher speed
        MINIMUM_SPEEDING_STEPS = 6    
        if (steps <= MINIMUM_SPEEDING_STEPS) and (speed>= 8.5):
            reward = 1
        else:
            reward = 1e-4
    
        # We use two previous progress parameter, so the minimum step the car
        # must run before we can calculate the reward is 3
        # This will give reward depend on the car's progress increase speed, 
        # so if car increase progress faster, it will get higher reward.
        # In other words, the faster you reach the finishing line, the higher
        # reward you'll get
        if (steps > MINIMUM_SPEEDING_STEPS):
            reward = ((progress- self.pre_progress)*2)**2 + \
                     ((progress- self.pre_progress2))**2
            
        # Sometime in local training, car teleport to random location, this 
        # will lead to a huge progress reward (from 10% progress jumped to 20%
        # progress) and therefore disrupts our model. Therefore we need this 
        # to remove the teleport problem
        if ((progress - self.pre_progress)>=5) or ((progress - self.pre_progress2)>=5):
            reward = 1e-4
            
        # Avoid negative reward
        if (reward <= 0):
            reward = 1e-4
        
        if self.debug:
            # Print the parameter out to console (so we can see it in local training)                
            carString=('REW: {:3.1f}, PRO: {:5.2f}, PRE_PRO: {:5.2f}, '
                   'PRE_PRO2: {:5.2f}, STEER:{:5.1f}, SPEED: {:3.1f}, '
                   'IS_LEFT: {}, NORMDIST: {:.2f}, DISTANCE: {:.2f}, '
                   ' TRACK_WIDTH: {:.2f}'.format(reward, progress, 
                   self.pre_progress, self.pre_progress2, steering_angle, 
                   speed, is_left_of_center, normDistance, 
                   distance_from_center, track_width))
            print(carString)       
        
        self.pre_progress2 = self.pre_progress
        self.pre_progress = progress        
        self.previous_steps = steps
        if self.debug:
            # Print the parameter out to console (so we can see it in local training)                
            carString=('REW: {:3.1f}, PRO: {:5.2f}, PRE_PRO: {:5.2f}, '
                   'PRE_PRO2: {:5.2f}, STEER:{:5.1f}, SPEED: {:3.1f}, '
                   'IS_LEFT: {}, NORMDIST: {:.2f}, DISTANCE: {:.2f}, '
                   ' TRACK_WIDTH: {:.2f}, reward: {:.2f}'.format(reward, progress, 
                   self.pre_progress, self.pre_progress2, steering_angle, 
                   speed, is_left_of_center, normDistance, 
                   distance_from_center, track_width, reward))
            print(carString)
        return float(reward)

myCarObject = MyCar(True)
        
def reward_function(params):
    return myCarObject.reward_function(params)    