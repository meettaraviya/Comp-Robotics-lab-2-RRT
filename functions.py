import numpy as np
import pygame
from pygame.locals import *

# C                B       A
# |----------------========|
# |                        |
# |                        |
# |                   o    | ---> Front when h = 0  
# |                        |
# |                        |
# |----------------========|
# F                E       D
# 
# o = Origin


# Function Defintions 
# map origin: bottom left corner
# state s = (x, y, h)
# 	units = (mm, mm, rad)

# Global Variables
map_width = 1000.0  #mm
map_length = 1000.0  #mm

robot_width = 90.0  #mm
robot_height = 100.0  #mm
wheel_radius = 25.0  #mm

max_rpm = 60  # rpm
robot_circumference = 2*np.pi*(robot_width/2)
wheel_circumference = 2*np.pi*wheel_radius

point_size = 10

# [A, B, C, D, E, F]
robot_points = [
	(+wheel_radius , -robot_width/2 ),
	(-wheel_radius , -robot_width/2 ),
	(-(robot_height - wheel_radius) , -robot_width/2 ),
	(+wheel_radius , +robot_width/2 ),
	(-wheel_radius , +robot_width/2 ),
	(-(robot_height - wheel_radius) , +robot_width/2 ),
]

fps = 30.0

velocity = wheel_circumference * max_rpm * (1/60.0)  # mm/sec = mm/rev * rev/min * min/sec
angular_speed = velocity / (robot_width / 2)	# rad/sec = mm/sec * rad/mm

precision = 1e-3
PI = np.pi

# # oA
# small_turn_radius = np.sqrt(wheel_radius**2 + (robot_width/2)**2)
# # oB
# large_turn_radius = np.sqrt((robot_height - wheel_radius)**2 + (robot_width/2)**2)
# # directions of A,C,D,F
# corner_angles = [np.arctan2(*robot_points[0]), np.arctan2(*robot_points[2]), np.arctan2(*robot_points[3]), np.arctan2(*robot_points[5])]


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
	drive_direction = 1
	if (abs(delta1) + abs(delta2)) < (abs(delta1_back) + abs(delta2_back)):
		delta = [delta1, delta2]
	else:
		delta = [delta1_back, delta2_back]
		h_driving = h_driving_back
		drive_direction = -1

	# store the rotation times and decrease the remaining time
	rotate_time1 = angle2time(delta[0])
	remaining_time -= rotate_time1

	rotate_time2 = angle2time(delta[1])
	remaining_time -= rotate_time2

	# determine how far the robot can travel within the remaining time
	move_time = drive_direction * remaining_time  # move_time < 0 if driving backwards
	max_dist = velocity * move_time

	# if the robot can reach the target in time, drive there; otherwise, drive max_dist
	target_dist = np.sqrt( (y_target - y_current)**2 + (x_target - x_current)**2 )
	dist = target_dist if target_dist < max_dist else max_dist

	x_new = x_current + ( drive_direction * dist*np.cos(h_driving) )
	y_new = y_current + ( drive_direction * dist*np.sin(h_driving) )
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
	return False

# Meet
def check_target(s_start, s_end, action, target):
	# If the path passes through the target with similar direction as target heading,
	# return a new action; otherwise, return the same original action
	tx, ty, th = target
	r0, m0, r1 = action

	six, siy, sih = s_start
	sfx, sfy, sfh = s_end

	# dist_si_t = np.sqrt((six-tx)**2+(siy-ty)**2)

	# t_ray = np.atan2(ty-siy, tx-six)
	sf_ray = np.atan2(sfy-siy, sfx-six)

	dist_si_sf = abs(m0*velocity)

	perp_dist = abs(((sfx-six)*(ty-sfy)-(tx-sfx)*(sfy-siy))/dist_si_sf)
	if perp_dist < robot_width/2:
		proj_frac = ((tx-six)*(sfx-six)+(ty-siy)*(sfy-siy))/(dist_si_sf**2)
		projx = six + proj_frac*(sfx-six)
		projy = siy + proj_frac*(sfy-siy)

		if (six - projx)*(sfx - projx) < 0 and np.cos(th-sf_ray)>0:
			return True, (r0, m0*proj_frac, 0)

	return False, action


# Yuanyuan
def display_edge(edge):
	return


# Meet
def print_path(S, E, s_initial, target, obstacles):
	visited = {s:False for s in S}
	prev = {}

	queue = []

	queue.append(s_initial)
	visited[s_initial] = True

	while queue: 

		# Dequeue a vertex from  
		# queue and print it 
		s = queue.pop(0) 

		for u, v, a in E:
			if u == s and visited[v] == False:
				queue.append(v)
				visited[v] = True
				prev[v] = (u,v,a)

				if v == target:
					queue = []
					break

	if v != target:
		print("No path found yet")
		return

	u = target
	state_seq = [target]
	action_seq = []

	while u != s_initial:

		u, v, a = prev[u]
		state_seq.append(u)
		action_seq.append(a)

	action_seq = list(reversed(action_seq))
	action_seq.append(None)

	state_seq = list(reversed(state_seq))

	print(state_seq)
	print(action_seq)

	frame_seq = []

	for state, action in zip(state_seq, action_seq):

		if action is None:
			frame_seq.append(state)
			break

		r0, m0, r1 = action
		x, y, h = state


		for i in range(round(abs(fps * r0))):
			frame_seq.append((x, y, h + np.sign(r0) * (angular_speed / fps) * i))

		h_r0 = h + angular_speed * r0
		
		dxpf = velocity * np.cos(h_r0) / fps
		dypf = velocity * np.sin(h_r0) / fps

		for i in range(round(abs(fps * m0))):
			frame_seq.append((x + i*dxpf, y + np.sign(m0) * i * dypf, h_r0))

		xf = x + velocity * np.cos(h_r0) * m0
		yf = y + velocity * np.sin(h_r0) * m0

		for i in range(round(abs(fps * r1))):
			frame_seq.append((xf, yf , h_r0 + np.sign(r1) * (angular_speed / fps) * i))

		h_r1 = h_r0 + angular_speed * r1

	return frame_seq


def draw_robot(screen, state):
	x, y, h = state
	print(x,y,h)
	t_points = [
		(x+p[0]*np.cos(h)-p[1]*np.sin(h), y+p[0]*np.sin(h)+p[1]*np.cos(h)) for p in robot_points
	]
	pygame.draw.line(screen, (255,0,0), t_points[0], t_points[2])
	pygame.draw.line(screen, (255,0,0), t_points[3], t_points[5])
	pygame.draw.line(screen, (255,0,0), t_points[0], t_points[3])
	pygame.draw.line(screen, (255,0,0), t_points[2], t_points[5])

	pygame.draw.line(screen, (0,0,255), t_points[0], t_points[1], 3)
	pygame.draw.line(screen, (0,0,255), t_points[3], t_points[4], 3)



def play_frames(frames):
	pygame.init()
	screen = pygame.display.set_mode((round(map_width), round(map_length)))
	pygame.display.set_caption('RRT demo')

	# Fill background
	background = pygame.Surface(screen.get_size())
	background = background.convert()
	background.fill((255, 255, 255, 100))

	clock = pygame.time.Clock()
	clock.tick(round(fps))

	# Display some text
	font = pygame.font.Font(None, 36)
	text = font.render("Hello There", 1, (10, 10, 10))
	textpos = text.get_rect()
	textpos.centerx = background.get_rect().centerx
	background.blit(text, textpos)

	# Blit everything to the screen
	screen.blit(background, (0, 0))
	pygame.display.flip()

	# Event loop
	for frame in frame_seq:
		for event in pygame.event.get():
			if event.type == QUIT:
				return


		screen.blit(background, (0, 0))
		pygame.draw.circle(screen, (0,255,0), (round(target[0]), round(target[1])), point_size)
		pygame.draw.line(screen, (0,255,0), (round(target[0]), round(target[1])), (round(target[0]+2*point_size*np.cos(target[2])), round(target[1]+2*point_size*np.sin(target[2]))), 4)
		draw_robot(screen, frame)
		pygame.display.flip()
		clock.tick(round(fps))



# pseudocode
def RRT(s_initial, target, obstacles):
	S = [s_initial]
	E = []
	# target = (x_target, y_target, h_target)
	# obstacles = [ (p1, p2, p3, p4) ]

	while(1):

		s_rand = (np.random.rand()*map_width, np.random.rand()*map_length, np.random.rand()*2*np.pi) 

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



	print_path(S, E, s_initial, target, obstacles)

	return

def RRT_star():
	return


if __name__ == "__main__":
	start = (0,0,PI)
	end = (1000,0,PI)
	[state, action] = drive_towards(start, end)
	print("Target:", end)
	print("Final:", state)
	print("Times:", action)

	# RRT((map_width/2, map_length/2, 0), (3*map_width/4, 3*map_length/4, 0), [])

	# # Initialise screen
	# pygame.init()
	# screen = pygame.display.set_mode((round(map_width), round(map_length)))
	# pygame.display.set_caption('RRT demo')

	# # Fill background
	# background = pygame.Surface(screen.get_size())
	# background = background.convert()
	# background.fill((250, 250, 250))

	# # Display some text
	# font = pygame.font.Font(None, 36)
	# text = font.render("Hello There", 1, (10, 10, 10))
	# textpos = text.get_rect()
	# textpos.centerx = background.get_rect().centerx
	# background.blit(text, textpos)

	# # Blit everything to the screen
	# screen.blit(background, (0, 0))
	# pygame.display.flip()

	# # Event loop
	# while 1:
	# 	for event in pygame.event.get():
	# 		if event.type == QUIT:
	# 			exit(0)

	# 	screen.blit(background, (0, 0))
	# 	pygame.display.flip()

	S = [
		(500, 500, np.pi/2),
		(500+50*np.pi, 500, 0),
		(500+50*np.pi, 500, np.pi/2),
		(500+50*np.pi, 500+50*np.pi, np.pi/2),
		(500+50*np.pi, 500, 3*np.pi/2),
	]

	E = [
		(S[0], S[1], (-0.45, +1.  ,  0.  )),
		(S[0], S[2], (-0.45, +1.  ,  0.45)),
		(S[2], S[3], ( 0.  , +1.  ,  0.  )),
		(S[2], S[4], ( 0.  ,  0.01, -0.90)),
	]

	s_initial = S[0]
	target = S[4]

	obstacles = []

	frame_seq = print_path(S, E, s_initial, target, obstacles)
	play_frames(frame_seq)
