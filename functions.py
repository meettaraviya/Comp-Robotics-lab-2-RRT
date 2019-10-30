# Function Defintions 

# map origin: bottom left corner
# state s = (x, y, h)
# 	units = (mm, mm, rad)

map_width = 1000.0  #mm
map_length = 1000.0  #mm

max_rpm = 60  # rpm
precision = 1e-3


# Adrian
def metric(s1, s2):
	# Use a metric to determine the distance between states s1 and s2
	return distance

# Adrian
def nearest_neighbor(S, s_rand):
	# Use the metric function to determine which state in S is closest to s_rand
	return s_nearest


# Andrew
def drive_towards(s_start, s_rand, max_time=1):
	# Drive from state s_start towards s_rand for max_time seconds

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
		s_rand = rand()

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
	RRT()


