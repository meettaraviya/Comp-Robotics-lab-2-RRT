import numpy as np
import pygame
from pygame.locals import *
import math
from check_collision import *
import math
import shapely
from shapely.geometry import Polygon
from shapely import affinity
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

np.random.seed(0)

playspeed = 1

map_width = 1000.0  #mm
map_length = 500.0  #mm

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

######

max_rotation_speed = angular_speed  #radians per sec
max_translation_speed = velocity    #mm per sec

######
precision = 1e-3
PI = np.pi


def check_collision(s_start, s_end, action, obstacles):
	# Given a straight line path from state s_start to state s_end, check if the path intersects with any obstacles
	# Also check if rotations result in collisions with obstacles
	# This can also be used to check if the robot entered the target space

	# obstacles = [(p1, p2, p3, p4), (p1, p2, p3, p4), ...]
	# ^this is a list of tuples defining the four corners of a rectangular obstacle
	# s1        s2
	# 
	# s4        s3

	# True: there is a collision
	# False: there is no collision
	
	
	x_start = s_start[0]
	y_start = s_start[1]
	h_start = s_start[2]
	
	x_end = s_end[0]
	y_end = s_end[1]
	h_end = s_end[2]

	L = map_length
	W = map_width
	
	bound_1 = Polygon([ (-0.1,-0.1), (-0.1,0),     (L+0.1,0),     (L+0.1,-0.1) ])
	bound_2 = Polygon([ (-0.1,W),    (-0.1,W+0.1), (L+0.1,W+0.1), (L+0.1,W)    ])
	bound_3 = Polygon([ (-0.1,-0.1), (-0.1,W+0.1), (0,W+0.1),     (0,-0.1)     ])
	bound_4 = Polygon([ (L,-0.1),    (L,W+0.1),    (L+0.1,W+0.1), (L+0.1,-0.1) ])
	# print('pass1')
	# First Rotation
	start_p1_x = (-45)*math.cos(h_start-0.5*math.pi) - (-75)*math.sin(h_start-0.5*math.pi) + x_start
	start_p2_x = (-45)*math.cos(h_start-0.5*math.pi) - (25)*math.sin(h_start-0.5*math.pi) + x_start
	start_p3_x = (45)*math.cos(h_start-0.5*math.pi) - (25)*math.sin(h_start-0.5*math.pi) + x_start
	start_p4_x = (45)*math.cos(h_start-0.5*math.pi) - (-75)*math.sin(h_start-0.5*math.pi) + x_start
 
	start_p1_y = (-45)*math.sin(h_start-0.5*math.pi) + (-75)*math.cos(h_start-0.5*math.pi) + y_start
	start_p2_y = (-45)*math.sin(h_start-0.5*math.pi) + (25)*math.cos(h_start-0.5*math.pi) + y_start
	start_p3_y = (45)*math.sin(h_start-0.5*math.pi) + (25)*math.cos(h_start-0.5*math.pi) + y_start
	start_p4_y = (45)*math.sin(h_start-0.5*math.pi) + (-75)*math.cos(h_start-0.5*math.pi) + y_start
	
	a = Polygon([(start_p1_x, start_p1_y), (start_p2_x, start_p2_y), (start_p3_x,start_p3_y), (start_p4_x, start_p4_y)])
	union_a = a
	for angle in np.linspace(0,2*math.pi*action[0],1000):
		rotated = affinity.rotate(a, angle, origin=(x_start,y_start), use_radians=True)
		union_a = union_a.union(rotated)
		
	# check collision with obstacles
	for obstacle in obstacles:
		b = Polygon(obstacle)
		if union_a.intersection(b).area > 0:
			return True
	# check collision with boundaries
	if union_a.intersection(bound_1).area > 0:
		return True
	if union_a.intersection(bound_2).area > 0:
		return True
	if union_a.intersection(bound_3).area > 0:
		return True
	if union_a.intersection(bound_4).area > 0:
		return True
	
	
	# print('pass rotation1')
	# straight line with obstacles
	straight_line_slope = (y_end - y_start)/(x_end - x_start)
	straight_line_slope = y_end - straight_line_slope*x_end
	
	# find rotation angle counter clockwise
	if straight_line_slope >= 0:
		if x_start <= x_end:
			rotating_angle = 1.5*math.pi + math.atan(straight_line_slope)
		else:
			rotating_angle = 0.5*math.pi + math.atan(straight_line_slope)
	else: 
		if x_start <= x_end:
			rotating_angle = 1.5*math.pi - math.atan(abs(straight_line_slope))
		else:
			rotating_angle = 0.5*math.pi - math.atan(abs(straight_line_slope))
	
	startL_x,startR_x,endL_x,endR_x = -45,45,-45,45
	if action[1] >= 0: # move_forward
		startL_y,startR_y,endL_y,endR_y = -75,-75,25,25
	else:
		startL_y,startR_y,endL_y,endR_y = 25,25,-75,-75
			   
	point1_x = startL_x*math.cos(rotating_angle) - startL_y*math.sin(rotating_angle) + x_start
	point2_x = startR_x*math.cos(rotating_angle) - startR_y*math.sin(rotating_angle) + x_start
	point3_x = endR_x*math.cos(rotating_angle) - endR_y*math.sin(rotating_angle) + x_end
	point4_x = endL_x*math.cos(rotating_angle) - endL_y*math.sin(rotating_angle) + x_end
	
	point1_y = startL_x*math.sin(rotating_angle) + startL_y*math.cos(rotating_angle) + y_start
	point2_y = startR_x*math.sin(rotating_angle) + startR_y*math.cos(rotating_angle) + y_start
	point3_y = endR_x*math.sin(rotating_angle) + endR_y*math.cos(rotating_angle) + y_end
	point4_y = endL_x*math.sin(rotating_angle) + endL_y*math.cos(rotating_angle) + y_end
	   
	a = Polygon([(point1_x, point1_y), (point2_x, point2_y), (point3_x,point3_y), (point4_x, point4_y)])
	# check collision with obstacles
	for obstacle in obstacles:
		b = Polygon(obstacle)
		if a.intersection(b).area > 0:
			return True
	# check collision with boundaries
	if a.intersection(bound1).area > 0:
		return True
	if a.intersection(bound2).area > 0:
		return True
	if a.intersection(bound3).area > 0:
		return True
	if a.intersection(bound4).area > 0:
		return True
	
	
	# Second Rotation
	end_p1_x = (-45)*math.cos(rotating_angle) - (-75)*math.sin(rotating_angle) + x_end
	end_p2_x = (-45)*math.cos(rotating_angle) - (25)*math.sin(rotating_angle) + x_end
	end_p3_x = (45)*math.cos(rotating_angle) - (25)*math.sin(rotating_angle) + x_end
	end_p4_x = (45)*math.cos(rotating_angle) - (-75)*math.sin(rotating_angle) + x_end
 
	end_p1_y = (-45)*math.sin(rotating_angle) + (-75)*math.cos(rotating_angle) + y_end
	end_p2_y = (-45)*math.sin(rotating_angle) + (25)*math.cos(rotating_angle) + y_end
	end_p3_y = (45)*math.sin(rotating_angle) + (25)*math.cos(rotating_angle) + y_end
	end_p4_y = (45)*math.sin(rotating_angle) + (-75)*math.cos(rotating_angle) + y_end
	
	a = Polygon([(end_p1_x, end_p1_y), (end_p2_x, end_p2_y), (end_p3_x,end_p3_y), (end_p4_x, end_p4_y)])
	union_a = a
	for angle in np.linspace(0,2*math.pi*action[0],1000):
		rotated = affinity.rotate(a, angle, origin=(x_end,y_end), use_radians=True)
		union_a = union_a.union(rotated)
		
	# check collision with obstacles
	for obstacle in obstacles:
		b = Polygon(obstacle)
		if union_a.intersection(b).area > 0:
			return True
	# check collision with boundaries
	if union_a.intersection(bound1).area > 0:
		return True
	if union_a.intersection(bound2).area > 0:
		return True
	if union_a.intersection(bound3).area > 0:
		return True
	if union_a.intersection(bound4).area > 0:
		return True
	
	return False


# Meet
def transition(s1, a):
	s1x, s1y, s1h = s1
	r0, m0, r1 = a

	s1h_ = s1h + r0 * angular_speed

	s1x_ = s1x + np.cos(s1h_) * velocity * m0
	s1y_ = s1y + np.sin(s1h_) * velocity * m0

	s1h__ = s1h_ + r1 * angular_speed

	return (s1x_, s1y_, s1h__)
		
# Meet
def metric(s1, s2):
	# Use a metric to determine the distance between states s1 and s2
	#result = [(time1(+/-), ang1(+/-), ( time2, dist2 (+)), (time3(+/-), rotation3(+/-))]
	s1x, s1y, s1h = s1
	s2x, s2y, s2h = s2

	mh = np.arctan2(s2y-s1y, s2x-s1x)

	t1 = np.sqrt((s2x-s1x)**2+(s2y-s1y)**2) / velocity

	if t1 > precision:
		t0 = min((s1h - mh) % (2*np.pi), (mh - s1h) % (2*np.pi)) / angular_speed
		t2 = min((s2h - mh) % (2*np.pi), (mh - s2h) % (2*np.pi)) / angular_speed
	else:
		t0 = min((s1h - s2h) % (2*np.pi), (s2h - s1h) % (2*np.pi)) / angular_speed
		t2 = 0

	return t0 + t1 + t2

# Adrian
def nearest_neighbor(S, s_rand):
	nearest_node = (S[0], metric(S[0], s_rand))

	for i in S: 
		if (metric(i,s_rand) < nearest_node[1]):
			nearest_node =  (i, metric(i, s_rand) )

	# Use the metric function to determine which state in S is closest to s_rand
	return nearest_node[0]


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

# Meet
def drive_towards(s_start, s_rand, max_time=1):
	
	sx, sy, sh = s_start
	tx, ty, th = s_rand

	lineh = np.arctan2(ty-sy, tx-sx)

	time_left = max_time

	r0o1 = (lineh - sh) % (2*np.pi) # cc-wise
	r0o2 = (sh - lineh) % (2*np.pi) # c-wise

	t0 = r0o1/angular_speed if r0o1 < r0o2 else -r0o2/angular_speed

	if abs(t0) >= time_left:
		t0 = np.sign(t0) * time_left
		a = (t0, 0, 0)
		# assert abs(a[0])+abs(a[1])+abs(a[2]) <= max_time
		return transition(s_start, a), a

	time_left -= abs(t0)

	dist = np.sqrt((tx-sx)**2 + (ty-sy)**2)
	t1 = dist/velocity

	if t1 >= time_left:
		t1 = time_left
		a = (t0, t1, 0)
		# assert abs(a[0])+abs(a[1])+abs(a[2]) <= max_time
		return transition(s_start, a), a

	time_left -= t1

	r2o1 = (th - lineh) % (2*np.pi) # cc-wise
	r2o2 = (lineh - th) % (2*np.pi) # c-wise

	t2 = r2o1/angular_speed if r2o1 < r2o2 else -r2o2/angular_speed

	if abs(t2) >= time_left:
		t2 = np.sign(t2) * time_left
	
	a = (t0, t1, t2)
	# assert abs(a[0])+abs(a[1])+abs(a[2]) <= max_time
	return transition(s_start, a), a


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
	sf_ray = np.arctan2(sfy-siy, sfx-six)

	dist_si_sf = abs(m0*velocity)

	perp_dist = abs(((sfx-six)*(ty-sfy)-(tx-sfx)*(sfy-siy))/dist_si_sf)
	if perp_dist < robot_width/2:
		proj_frac = ((tx-six)*(sfx-six)+(ty-siy)*(sfy-siy))/(dist_si_sf**2)
		projx = six + proj_frac*(sfx-six)
		projy = siy + proj_frac*(sfy-siy)

		if (six - projx)*(sfx - projx) < 0 and np.cos(th-sf_ray)>0:
			return True, (r0, m0*proj_frac, 0)

	return False, action


# Meet
def display_edge(edge, flip=True, color=(0,0,0)):
	pygame.draw.line(screen, color, edge[0][:2], edge[1][:2])
	pygame.draw.circle(screen, color, (round(float(edge[1][0])), round(float(edge[1][1]))), round(point_size/2))
	if flip:
		pygame.display.flip()

# contains final path
action_seq = []
state_seq = []


def draw_obstacles(obstacles):

	for obstacle in obstacles:
		pygame.draw.polygon(screen, (0,0,255), obstacle)

# Meet
def print_path(S, E, s_initial, target, obstacles):
	visited = {s:False for s in S}
	prev = {}

	queue = []

	queue.append(s_initial)
	visited[s_initial] = True

	global state_seq, action_seq

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
		return []

	u = target
	state_seq.append(target)

	while u != s_initial:

		u, v, a = prev[u]
		state_seq.append(u)
		action_seq.append(a)

	action_seq = list(reversed(action_seq))
	action_seq.append(None)
	# print(*action_seq)

	state_seq = list(reversed(state_seq))

	# print(state_seq)
	# print(action_seq)

	frame_seq = []

	for state, action in zip(state_seq, action_seq):

		if action is None:
			frame_seq.append(state)
			break

		r0, m0, r1 = action
		x, y, h = state


		for i in range(round(abs(float(fps * r0)))):
			frame_seq.append((x, y, h + np.sign(r0) * (angular_speed / fps) * i))

		h_r0 = h + angular_speed * r0
		
		dxpf = velocity * np.cos(h_r0) / fps
		dypf = velocity * np.sin(h_r0) / fps

		for i in range(round(abs(float(fps * m0)))):
			frame_seq.append((x + np.sign(m0) * i * dxpf, y + np.sign(m0) * i * dypf, h_r0))

		xf = x + velocity * np.cos(h_r0) * m0
		yf = y + velocity * np.sin(h_r0) * m0

		for i in range(round(abs(float(fps * r1)))):
			frame_seq.append((xf, yf , h_r0 + np.sign(r1) * (angular_speed / fps) * i))

		h_r1 = h_r0 + angular_speed * r1

	return frame_seq


def draw_robot(screen, state):
	x, y, h = state
	# print(x,y,h)
	t_points = [
		(x+p[0]*np.cos(h)-p[1]*np.sin(h), y+p[0]*np.sin(h)+p[1]*np.cos(h)) for p in robot_points
	]
	pygame.draw.line(screen, (255,0,0), t_points[0], t_points[2])
	pygame.draw.line(screen, (255,0,0), t_points[3], t_points[5])
	pygame.draw.line(screen, (255,0,0), t_points[0], t_points[3])
	pygame.draw.line(screen, (255,0,0), t_points[2], t_points[5])

	pygame.draw.line(screen, (0,0,255), t_points[0], t_points[1], 3)
	pygame.draw.line(screen, (0,0,255), t_points[3], t_points[4], 3)



def play_frames(frames, target, obstacles):

	# Fill background
	background = pygame.Surface(screen.get_size())
	background = background.convert()
	background.fill((255, 255, 255, 100))

	clock = pygame.time.Clock()
	clock.tick(round(fps * playspeed))

	# # Display some text
	# font = pygame.font.Font(None, 36)
	# text = font.render("Hello There", 1, (10, 10, 10))
	# textpos = text.get_rect()
	# textpos.centerx = background.get_rect().centerx
	# background.blit(text, textpos)

	# Blit everything to the screen
	screen.blit(background, (0, 0))
	pygame.display.flip()

	L = len(state_seq)
	edges = []
	for i in range(L-1):
		edges.append((state_seq[i], state_seq[i+1]))

	# Event loop
	for frame in frames:
		for event in pygame.event.get():
			if event.type == QUIT:
				return


		screen.blit(background, (0, 0))

		for edge in edges:
			display_edge(edge, flip=False)

		draw_obstacles(obstacles)

		pygame.draw.circle(screen, (0,255,0), (round(target[0]), round(target[1])), point_size)
		pygame.draw.line(screen, (0,255,0), (round(target[0]), round(target[1])), (round(target[0]+2*point_size*np.cos(target[2])), round(target[1]+2*point_size*np.sin(target[2]))), 4)
		draw_robot(screen, frame)
		pygame.display.flip()
		clock.tick(round(fps * playspeed))



# pseudocode
def RRT(s_initial, target, obstacles):
	S = [s_initial]
	E = []
	# target = (x_target, y_target, h_target)
	obstacles = [ 
		(
			(0.6*map_width,0.3*map_length),
			(0.6*map_width,0.7*map_length),
			(0.65*map_width,0.7*map_length),
			(0.65*map_width,0.3*map_length)
			)
		]

	draw_obstacles(obstacles)

	while(1):

		import time
		time.sleep(0.1)

		s_rand = (np.random.rand()*map_width, np.random.rand()*map_length, np.random.rand()*2*np.pi) 

		s_nearest = nearest_neighbor(S, s_rand)

		s_new, action = drive_towards(s_nearest, s_rand)

		if check_collision(s_nearest, s_new, action, obstacles):
			display_edge( (s_nearest, s_new, action), color=(255,0,0))
			continue
		else: 
			found_target, new_action = check_target(s_nearest, s_new, action, target)

			s_new = target if found_target else s_new
			S.append(s_new)
			# check_transition(s_nearest, action, s_new)
			E.append( (s_nearest, s_new, new_action) )

			display_edge( (s_nearest, s_new, new_action) )

			if found_target:
				time.sleep(2)
				break



	frames = print_path(S, E, s_initial, target, obstacles)
	# print(s_initial)
	play_frames(frames, target, obstacles)

	return

def RRT_star():
	return


if __name__ == "__main__":

	start = (map_width/2,map_length/2,PI)
	end = (3*map_width/4,3*map_length/4,PI/2)

	
	pygame.init()
	screen = pygame.display.set_mode((round(map_width), round(map_length)))
	pygame.display.set_caption('RRT demo')
	background = pygame.Surface(screen.get_size())
	background = background.convert()
	background.fill((255, 255, 255, 100))
	screen.blit(background, (0, 0))

	pygame.draw.circle(screen, (0,255,0), (round(end[0]), round(end[1])), point_size)

	RRT(start, end, [])

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

	# frame_seq = print_path(S, E, s_initial, target, obstacles)
	# play_frames(frame_seq)
