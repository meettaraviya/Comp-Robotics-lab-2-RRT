import math
import shapely
from shapely.geometry import Polygon
from shapely import affinity
import numpy as np
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
    
    bound_1 = Polygon([ (-0.1,-0.1), (-0.1,0),     (L+0.1,0),     (L+0.1,-0.1) ])
    bound_2 = Polygon([ (-0.1,W),    (-0.1,W+0.1), (L+0.1,W+0.1), (L+0.1,W)    ])
    bound_3 = Polygon([ (-0.1,-0.1), (-0.1,W+0.1), (0,W+0.1),     (0,-0.1)     ])
    bound_4 = Polygon([ (L,-0.1),    (L,W+0.1),    (L+0.1,W+0.1), (L+0.1,-0.1) ])
    print('pass1')
    # First Rotation
    start_p1_x = (-45)*math.cos(2*math.pi-h_start) - (-75)*math.sin(2*math.pi-h_start) + x_start
    start_p2_x = (-45)*math.cos(2*math.pi-h_start) - (25)*math.sin(2*math.pi-h_start) + x_start
    start_p3_x = (45)*math.cos(2*math.pi-h_start) - (25)*math.sin(2*math.pi-h_start) + x_start
    start_p4_x = (45)*math.cos(2*math.pi-h_start) - (-75)*math.sin(2*math.pi-h_start) + x_start
 
    start_p1_y = (-45)*math.sin(2*math.pi-h_start) + (-75)*math.cos(2*math.pi-h_start) + y_start
    start_p2_y = (-45)*math.sin(2*math.pi-h_start) + (25)*math.cos(2*math.pi-h_start) + y_start
    start_p3_y = (45)*math.sin(2*math.pi-h_start) + (25)*math.cos(2*math.pi-h_start) + y_start
    start_p4_y = (45)*math.sin(2*math.pi-h_start) + (-75)*math.cos(2*math.pi-h_start) + y_start
    
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
    if union_a.intersection(bound1).area > 0:
        return True
    if union_a.intersection(bound2).area > 0:
        return True
    if union_a.intersection(bound3).area > 0:
        return True
    if union_a.intersection(bound4).area > 0:
        return True
    
    
    print('pass rotation1')
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
    for angle in np.linspace(0,2*math.pi*action[0],100000):
        rotated = affinity.rotate(a, angle, origin=(x_start,y_start), use_radians=True)
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





