import numpy as np

# Function Defintions 
# map origin: bottom left corner
# state s = (x, y, h)
# 	units = (mm, mm, rad)


# Global Variables
map_width = 1000.0  #mm
map_length = 1000.0  #mm

robot_width = 90.0  #mm
wheel_radius = 25.0  #mm

max_rpm = 60  # rpm
robot_circumference = 2*np.pi*(robot_width/2)
wheel_circumference = 2*np.pi*wheel_radius

velocity = wheel_circumference * max_rpm * (1/60.0)  # mm/sec = mm/rev * rev/min * min/sec
angular_speed = velocity * (2*np.pi / robot_circumference)    # rad/sec = mm/sec * rad/mm

precision = 1e-3
PI = np.pi


# Adrian
def metric(s1, s2):
	# Use a metric to determine the distance between states s1 and s2
	return distance

# Adrian
def nearest_neighbor(S, s_rand):
	# Use the metric function to determine which state in S is closest to s_rand
	return s_nearest


def angle2time(angle):
	# Return time required for robot to rotate angle radians

	rotation_time = angle / angular_speed
	return rotation_time

def align_heading(h_start, h_end):
	# Return the shortest change in angle needed go from h_start to h_end (clockwise: delta<0; counterclockwise: delta>0)
	delta = h_end - h_start
	sign = -1 if delta < 0 else 1
	if abs(delta) > np.pi:
		delta = -sign * ( (2*np.pi) - delta )  # rotate opposite direction
	return delta

# Andrew
def drive_towards(s_start, s_rand, max_time=1):
	# Drive from state s_start towards s_rand for max_time seconds
	x_current = s_start[0]
	y_current = s_start[1]
	h_current = s_start[2]

	x_target = s_rand[0]
	y_target = s_rand[1]
	h_target = s_rand[2]

	# 1. rotate towards the target
	# 2. drive towards the target
	# 3. match the target's heading

	remaining_time = max_time

	# compute the positive angle of a straight line connecting s_start and s_rand
	h_driving = np.arctan2([y_target - y_current], [x_target - x_current])[0]
	if h_driving < 0:
		h_driving += (2*PI)

	# rotate to face target point, taking the shortest path
	delta1 = align_heading(h_current, h_driving)
	
	# consider rotating to setup driving backwards
	sign = -1 if delta1 < 0 else 1
	delta1_back = -sign * (PI - abs(delta1))
	h_driving_back = (h_driving + PI) % (2*PI)

	# compute rotations needed to match the target heading
	delta2 = align_heading(h_driving, h_target)
	delta2_back = align_heading(h_driving_back, h_target)

	# check with driving direction results in the smallest total rotation
	backwards = 1
	if (abs(delta1) + abs(delta2)) < (abs(delta1_back) + abs(delta2_back)):
		delta = [delta1, delta2]
	else:
		delta = [delta1_back, delta2_back]
		h_driving = h_driving_back
		backwards = -1

	# store the rotation times and decrease the remaining time
	rotate_time1 = angle2time(delta[0])
	remaining_time -= rotate_time1

	rotate_time2 = angle2time(delta[1])
	remaining_time -= rotate_time2

	# determine how far the robot can travel within the remaining time
	move_time = remaining_time
	max_dist = velocity * move_time

	# if the robot can reach the target in time, drive there; otherwise, drive max_dist
	target_dist = np.sqrt( (y_target - y_current)**2 + (x_target - x_current)**2 )
	dist = target_dist if target_dist < max_dist else max_dist

	x_new = x_current + backwards*dist*np.cos(h_driving)
	y_new = y_current + backwards*dist*np.sin(h_driving)
	h_new = h_target

	s_new = (x_new, y_new, h_new)
	action = (rotate_time1, move_time, rotate_time2)

	# action = (rotate_time, move, rotate_time)
	# 	units = (+/- s) 
	return s_new, action


# Yuanyuan
def check_collision(s_start, s_end, action, obstacles):
	# Given a straight line path from state s_start to state s_end, check if the path intersects with any obstacles
	# Also check if rotations result in collisions with obstacles
	# This can also be used to check if the robot entered the target space

	# obstacles = [(p1, p2, p3, p4), (p1, p2, p3, p4), ...]
	# 	^this is a list of tuples defining the four corners of a rectangular obstacle
	# 	s1		s2
	# 
	# 	s4		s3

	# True: there is a collision
	# False: there is no collision
	return True/False

# Meet
def check_target(s_start, s_end, action, target):
	# If the path passes through the target, return a new action
	# otherwise, return the same original action
	return True/False, new_action


# Yuanyuan
def display_edge(edge):
	return


# Meet
def print_path(S, E, s_initial, target):
	return


# pseudocode
def RRT():
	S = [s_initial]
	E = []
	target = (x_target, y_target, h_target)
	obstacles = [ (p1, p2, p3, p4) ]

	while(1):
		s_rand = (x_rand, y_rand, h_rand) 

		s_nearest = nearest_neighbor(S, s_rand)

		[s_new, action] = drive_towards(s_nearest, s_rand)

		if check_collision(s_nearest, s_new, action, obstacles):
			continue
		else: 
			[found_target, new_action] = check_target(s_nearest, s_new, action, obstacles)
			S.append(s_new)
			E.append( (s_nearest, s_new, new_action) )

			display_edge( (s_nearest, s_new, new_action) )

			if found_target:
				break


	print_path(S, E, s_initial, target)

	return

def RRT_star():
	return


if __name__ == "__main__":
	start = (0,0,0)
	end = (1000,0,0)
	[state, action] = drive_towards(start, end)
	print("Target:", end)
	print("Final:", state)
	print("Times:", action)
