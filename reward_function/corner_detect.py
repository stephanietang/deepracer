import math
import matplotlib.pyplot as plt
import numpy as np

TRACK_FILE = "C:\\Users\\PC\\workspace\\deepracer\\Capstone_AWS_DeepRacer\\Compute_Speed_And_Actions\\tracks\\arctic_open_ccw.npy"

# Parameters
FUTURE_STEP = 6
TURN_THRESHOLD = 10   # degrees

# Colour macros
FAST = 0
SLOW = 1

def identify_corner(waypoints, closest_waypoints, future_step):

    # Identify next waypoint and a further waypoint
    point_prev = waypoints[closest_waypoints[0]]
    point_next = waypoints[closest_waypoints[1]]
    point_future = waypoints[min(len(waypoints) - 1, 
                                 closest_waypoints[1] + future_step)]

    # Calculate headings to waypoints
    heading_current = math.degrees(math.atan2(point_prev[1]-point_next[1], 
                                              point_prev[0] - point_next[0]))
    heading_future = math.degrees(math.atan2(point_prev[1]-point_future[1], 
                                             point_prev[0]-point_future[0]))

    # Calculate the difference between the headings
    diff_heading = abs(heading_current - heading_future)

    # Check we didn't choose the reflex angle
    if diff_heading > 180:
        diff_heading = 360 - diff_heading

    # Calculate distance to further waypoint
    dist_future = np.linalg.norm([point_next[0] - point_future[0], 
                                  point_next[1] - point_future[1]])  

    return diff_heading, dist_future


# This is a modified version of the actual select_speed function used in 
# reward_qualifier.py so that there is a 3rd possible return value to allow 
# visualisation of the "bonus fast" points 
def select_speed(waypoints, closest_waypoints, future_step):

    # Identify if a corner is in the future
    diff_heading, dist_future = identify_corner(waypoints, closest_waypoints,
                                                future_step)
    
    if diff_heading < TURN_THRESHOLD:
        # If there's no corner encourage going faster
        speed_colour = FAST
    else:
        # If there is a corner encourage slowing down
        speed_colour = SLOW

    return speed_colour


# Get waypoints from numpy file
waypoints = np.load(TRACK_FILE)

print("----- Parameters -----")
print("   FUTURE_STEP: %d" % (FUTURE_STEP))
print("TURN_THRESHOLD: %d" % (TURN_THRESHOLD))
print("----------------------")

# Extract the x and y columns from the waypoints
waypoints = waypoints[:,2:4]
print(type(waypoints))

color_dict = {0:'#ff7f0e', 1:'#1f77b4', 2:'#ff460e'}
label_dict = {0:'Fast Incentive', 1:'Slow Incentive', 2:'Bonus Fast Incentive'}

colours = []

for i in range(len(waypoints)):
    # Simulate input parameter
    closest_waypoints = [i-1, i]

    # Determine what speed will be rewarded
    speed_colour = select_speed(waypoints, closest_waypoints, FUTURE_STEP)
    colours.append(speed_colour)

# Plot points
fig, ax = plt.subplots()

for g in np.unique(colours):
    ix = np.where(colours==g)
    ax.scatter(waypoints[ix,0], waypoints[ix,1], c=color_dict[g], 
               label=label_dict[g])

ax.legend(fancybox=True, shadow=True)
ax.set_aspect('equal')
plt.axis('off')

plt.show()